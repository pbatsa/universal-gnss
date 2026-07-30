#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "universal_gnss_transport/byte_stream.hpp"
#include "universal_gnss_transport/frame_writer.hpp"

namespace
{

using universal_gnss_transport::ByteSink;
using universal_gnss_transport::TransportError;
using universal_gnss_transport::TransportStatus;
using universal_gnss_transport::WriteFrameOptions;
using universal_gnss_transport::WriteFrameResult;
using universal_gnss_transport::WriteResult;

struct TestContext
{
  int failures{0};

  void Expect(const bool condition, const std::string& message)
  {
    if (!condition)
    {
      ++failures;
      std::cerr << "FAILED: " << message << '\n';
    }
  }
};

// A sink that accepts a scripted sequence of per-call outcomes so the retry and
// truncation paths can be exercised without a real serial port. A scripted
// `WriteResult` with `status == kOk` and `bytes_written == 0` models the
// non-blocking `EAGAIN` case (kernel TX buffer momentarily full).
class ScriptedSink : public ByteSink
{
public:
  explicit ScriptedSink(std::vector<WriteResult> script) : script_(std::move(script)) {}

  WriteResult Write(const std::uint8_t* data, const std::size_t size) override
  {
    ++calls;
    if (script_index_ >= script_.size())
    {
      // Default to accepting everything once the script is exhausted.
      accepted.insert(accepted.end(), data, data + static_cast<std::ptrdiff_t>(size));
      return WriteResult{size, TransportStatus::kOk, TransportError::kNone};
    }

    const auto scripted = script_[script_index_++];
    const std::size_t accept = std::min(scripted.bytes_written, size);
    accepted.insert(accepted.end(), data, data + static_cast<std::ptrdiff_t>(accept));
    return WriteResult{accept, scripted.status, scripted.error};
  }

  bool IsOpen() const override { return true; }
  void Close() override {}

  std::vector<std::uint8_t> accepted{};
  std::size_t calls{0u};

private:
  std::vector<WriteResult> script_{};
  std::size_t script_index_{0u};
};

WriteFrameOptions MakeTestOptions(const std::size_t max_stall_retries)
{
  WriteFrameOptions options;
  options.max_stall_retries = max_stall_retries;
  // Keep the tests instant: no real sleeping between retries.
  options.stall_backoff = std::chrono::microseconds{0};
  return options;
}

const std::vector<std::uint8_t> kFrame{0xD3u, 0x00u, 0x04u, 0x4Cu, 0xE0u, 0x00u, 0x80u, 0xEDu};

void TestCompleteWriteInOneCall(TestContext& ctx)
{
  ScriptedSink sink({});

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(4u));

  ctx.Expect(result.complete && !result.truncated, "a fully accepted frame should report complete");
  ctx.Expect(result.bytes_written == kFrame.size(),
             "a fully accepted frame should report every byte written");
  ctx.Expect(sink.accepted == kFrame, "the sink should receive the frame bytes unchanged");
  ctx.Expect(result.stall_retries == 0u, "an unstalled write should not report retries");
}

void TestPartialWritesAreResumedNotTruncated(TestContext& ctx)
{
  // Accept 3 bytes, then 2, then the remainder: a normal short write, which must
  // be resumed rather than treated as a failure.
  ScriptedSink sink({
      WriteResult{3u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{2u, TransportStatus::kOk, TransportError::kNone},
  });

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(4u));

  ctx.Expect(result.complete && !result.truncated,
             "short writes should be resumed until the whole frame is on the wire");
  ctx.Expect(result.bytes_written == kFrame.size(),
             "a resumed write should account for every frame byte");
  ctx.Expect(sink.accepted == kFrame,
             "a resumed write should deliver the frame bytes in order without gaps");
}

void TestStallIsRetriedThenCompletes(TestContext& ctx)
{
  // Two EAGAIN stalls mid-frame, then the transport drains and accepts the rest.
  // This is the case that previously put a TRUNCATED RTCM frame on the wire.
  ScriptedSink sink({
      WriteResult{4u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
  });

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(4u));

  ctx.Expect(result.complete && !result.truncated,
             "a transient stall mid-frame should be retried until the frame completes");
  ctx.Expect(sink.accepted == kFrame,
             "a retried stall must not drop the tail of the frame");
  ctx.Expect(result.stall_retries == 2u, "each stall retry should be reported for observability");
}

void TestStallRetryBudgetResetsOnProgress(TestContext& ctx)
{
  // One stall, progress, another stall, progress: with a budget of 1 retry this
  // must still succeed, because the budget applies per stall and not per frame.
  ScriptedSink sink({
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{4u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
  });

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(1u));

  ctx.Expect(result.complete && !result.truncated,
             "the stall retry budget should reset whenever the transport makes progress");
  ctx.Expect(sink.accepted == kFrame, "a frame stalled more than once should still complete");
}

void TestExhaustedStallBudgetReportsTruncation(TestContext& ctx)
{
  // A prefix goes out, then the transport never drains. The frame on the wire is
  // now corrupt, and the caller must be able to tell that apart from "nothing
  // was written".
  ScriptedSink sink({
      WriteResult{4u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
  });

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(2u));

  ctx.Expect(!result.complete, "an exhausted stall budget should not report completion");
  ctx.Expect(result.truncated,
             "a frame whose prefix reached the wire but could not be finished must be reported as "
             "truncated");
  ctx.Expect(result.bytes_written == 4u, "a truncated frame should report the prefix it wrote");
}

void TestStallBeforeAnyByteIsNotTruncation(TestContext& ctx)
{
  // Nothing reached the wire, so the receiver sees no partial frame. That is a
  // dropped frame, not a corrupt one.
  ScriptedSink sink({
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kOk, TransportError::kNone},
  });

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(1u));

  ctx.Expect(!result.complete, "a fully stalled write should not report completion");
  ctx.Expect(!result.truncated,
             "a write that never placed a byte on the wire is a dropped frame, not a truncated one");
  ctx.Expect(result.bytes_written == 0u, "a fully stalled write should report no bytes written");
}

void TestTransportErrorPropagates(TestContext& ctx)
{
  ScriptedSink sink({
      WriteResult{4u, TransportStatus::kOk, TransportError::kNone},
      WriteResult{0u, TransportStatus::kError, TransportError::kWriteFailure},
  });

  const auto result = universal_gnss_transport::WriteFrame(
      sink, kFrame.data(), kFrame.size(), MakeTestOptions(4u));

  ctx.Expect(!result.complete && result.status == TransportStatus::kError,
             "a hard transport error should surface as an error rather than a stall");
  ctx.Expect(result.error == TransportError::kWriteFailure,
             "a hard transport error should preserve the transport error code");
  ctx.Expect(result.truncated,
             "a hard error after a prefix reached the wire should still report truncation");
}

void TestEmptyAndNullInputsAreRejected(TestContext& ctx)
{
  ScriptedSink sink({});

  const auto empty_result =
      universal_gnss_transport::WriteFrame(sink, kFrame.data(), 0u, MakeTestOptions(4u));
  ctx.Expect(empty_result.complete && empty_result.bytes_written == 0u,
             "writing an empty frame should trivially succeed without touching the sink");
  ctx.Expect(sink.calls == 0u, "an empty frame should not reach the sink");

  const auto null_result =
      universal_gnss_transport::WriteFrame(sink, nullptr, kFrame.size(), MakeTestOptions(4u));
  ctx.Expect(!null_result.complete && null_result.error == TransportError::kInvalidArgument,
             "a null frame pointer should be rejected as an invalid argument");
}

}  // namespace

int main()
{
  TestContext ctx;

  TestCompleteWriteInOneCall(ctx);
  TestPartialWritesAreResumedNotTruncated(ctx);
  TestStallIsRetriedThenCompletes(ctx);
  TestStallRetryBudgetResetsOnProgress(ctx);
  TestExhaustedStallBudgetReportsTruncation(ctx);
  TestStallBeforeAnyByteIsNotTruncation(ctx);
  TestTransportErrorPropagates(ctx);
  TestEmptyAndNullInputsAreRejected(ctx);

  if (ctx.failures != 0)
  {
    std::cerr << ctx.failures << " test(s) failed\n";
    return EXIT_FAILURE;
  }

  std::cout << "All gnss_transport frame writer tests passed\n";
  return EXIT_SUCCESS;
}
