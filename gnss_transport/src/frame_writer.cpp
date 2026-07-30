#include "universal_gnss_transport/frame_writer.hpp"

#include <thread>

namespace universal_gnss_transport
{

namespace
{

WriteFrameResult MakeInvalidArgumentResult()
{
  WriteFrameResult result;
  result.status = TransportStatus::kError;
  result.error = TransportError::kInvalidArgument;
  return result;
}

}  // namespace

WriteFrameResult WriteFrame(ByteSink& sink,
                            const std::uint8_t* const data,
                            const std::size_t size,
                            const WriteFrameOptions& options)
{
  WriteFrameResult result;

  if (size == 0u)
  {
    result.complete = true;
    return result;
  }

  if (data == nullptr)
  {
    return MakeInvalidArgumentResult();
  }

  std::size_t offset = 0u;
  std::size_t consecutive_stalls = 0u;

  while (offset < size)
  {
    const auto attempt =
        sink.Write(data + static_cast<std::ptrdiff_t>(offset), size - offset);
    result.status = attempt.status;
    result.error = attempt.error;

    if (attempt.status != TransportStatus::kOk)
    {
      break;
    }

    if (attempt.bytes_written == 0u)
    {
      // Transport is momentarily unable to accept more (non-blocking EAGAIN).
      // Bail out only once the stall budget is spent, so a frame is not left
      // half-written just because the TX buffer filled up mid-frame.
      if (consecutive_stalls >= options.max_stall_retries)
      {
        break;
      }

      ++consecutive_stalls;
      ++result.stall_retries;
      if (options.stall_backoff.count() > 0)
      {
        std::this_thread::sleep_for(options.stall_backoff);
      }
      continue;
    }

    consecutive_stalls = 0u;
    offset += attempt.bytes_written;
    result.bytes_written = offset;
  }

  result.complete = offset == size;
  result.truncated = !result.complete && offset > 0u;
  return result;
}

}  // namespace universal_gnss_transport
