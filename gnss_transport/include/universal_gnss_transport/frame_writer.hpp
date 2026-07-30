#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "universal_gnss_transport/byte_stream.hpp"
#include "universal_gnss_transport/transport_error.hpp"
#include "universal_gnss_transport/transport_status.hpp"

namespace universal_gnss_transport
{

// Whole-frame write policy.
//
// A `ByteSink` backed by a non-blocking fd reports a momentarily full kernel TX
// buffer as `{bytes_written = 0, status = kOk}` (the POSIX `EAGAIN` path). A
// caller that treats that as "stop" leaves whatever prefix already went out on
// the wire, which for a framed protocol such as RTCM3 means the peer receives a
// TRUNCATED frame and fails its CRC. `WriteFrame` instead waits for the
// transport to drain so the frame either lands whole or is reported as damaged.
struct WriteFrameOptions
{
  // Consecutive zero-progress attempts tolerated before giving up. The budget
  // resets on every attempt that moves bytes, so it bounds a single stall
  // rather than the whole frame: a large frame trickling out over a slow UART
  // is not penalised.
  std::size_t max_stall_retries{64u};

  // Pause between consecutive zero-progress attempts. With the default budget
  // this bounds a single stall at roughly 64 ms, which comfortably covers
  // draining a full 4 KiB kernel TX buffer at 115200 bps (~11.5 B/ms) while
  // keeping the caller's thread responsive.
  std::chrono::microseconds stall_backoff{std::chrono::milliseconds{1}};
};

struct WriteFrameResult
{
  std::size_t bytes_written{0u};
  TransportStatus status{TransportStatus::kOk};
  TransportError error{TransportError::kNone};

  // Every byte of the frame reached the sink.
  bool complete{false};

  // A non-empty PREFIX of the frame reached the sink but the remainder did not.
  // The peer is now looking at a corrupt frame, which is materially worse than
  // a frame that was never sent: it is worth counting separately so a receiver
  // fighting a saturated link is diagnosable instead of merely "some write
  // errors".
  bool truncated{false};

  // Zero-progress attempts that were retried. Non-zero means the link is at or
  // near saturation even when the frame ultimately completed.
  std::size_t stall_retries{0u};
};

// Writes `size` bytes from `data` to `sink`, resuming across short writes and
// waiting out transient stalls. Returns as soon as the frame completes, the
// transport reports a hard error, or the stall budget is exhausted.
WriteFrameResult WriteFrame(ByteSink& sink,
                            const std::uint8_t* data,
                            std::size_t size,
                            const WriteFrameOptions& options = {});

}  // namespace universal_gnss_transport
