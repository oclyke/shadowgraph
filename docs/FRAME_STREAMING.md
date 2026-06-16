# Frame streaming: a buffered animation playout path

**Status:** design / proposal. Nothing implemented yet. This document is the
contract that the host sender and the firmware both build against. It sits on
top of the laser command stream (`GOTO`/`LASER`/`DWELL`/`CURVE`, see
`docs/CURVE_MOTION.md`) and the `laser_engine` consumer — it does **not** change
either.

## 1. Goal

Stream animations into the device without re-sending geometry every refresh.
A *frame* is a complete, self-contained scene (a blob of laser commands) that the
device **loops locally** until told to move on. The host pre-buffers frames; the
device plays exactly one *active* frame on repeat until an external tick advances
it. A static image therefore costs **zero ongoing traffic**; an animation costs
only the bytes of each new frame, once.

This decouples two concerns that do not belong on the same channel:

- **Delivery** — frame bytes must arrive *complete and in order*. Reliable, bulk,
  latency-tolerant. → **TCP.**
- **Playout timing** — "show the next frame *now*" must be low-jitter and must not
  block behind bulk data. → **UDP.**

The mental model is a **playout / jitter buffer driven by an external playout
clock** (like a frame-advance trigger separate from the video data).

## 2. Architecture & division of labor

```
          HOST (tools/…)                              FIRMWARE (ESP32, C)
 ┌───────────────────────────────┐         ┌──────────────────────────────────────┐
 │ plan each frame (svg2scene):  │         │ frame_buffer (NEW)                     │
 │   closed loop — starts AND    │  TCP    │  ┌──────────────────────────────────┐  │
 │   ends at center, v=0 at seam │ ──────▶ │  │ 96 KB arena: FIFO of frames      │  │
 │ frame the TCP stream:         │ (data)  │  │ forbidden wrap (PAD to lap)      │  │
 │   [magic][id][len][payload…]  │         │  │ cursors: write / committed /     │  │
 │                               │         │  │          playing                 │  │
 │ feed frames in PLAYOUT order  │         │  └──────────────────────────────────┘  │
 │   (repeats = re-send w/ new id)│        │         ▲ (sole writer)    │ (reader)   │
 │                               │         │   TCP recv task        pump task        │
 │ tick the playout clock:       │  UDP    │                            │            │
 │   NEXT  (relative tick)       │ ──────▶ │   UDP ctrl task → advance++ │           │
 └───────────────────────────────┘ (ctrl) │                            ▼            │
                                           │              laser_engine_*  (SPSC)     │
                                           │              → galvo / laser DACs       │
                                           └──────────────────────────────────────┘
```

Three firmware tasks, each with one job; **only the pump touches `laser_engine`**,
preserving its single-producer contract:

- **TCP recv task** — the *sole writer* of the arena. Reads framed frames off the
  TCP connection, places each contiguously, publishes it as committed.
- **UDP control task** — receives `NEXT` ticks and bumps a pending-advance counter.
  Never touches the arena bytes or the engine.
- **Pump task** — replays the active frame into `laser_engine` on repeat; advances
  the playout cursor at loop boundaries by the pending NEXT count.

## 3. The arena: a FIFO of variable-length frames

One **96 KB** continuous byte region. Frames are laid down back-to-back as
length-prefixed records; the sender chooses any mix of few-large or many-small
frames, and may change frame size at runtime simply by sending a different one.

**Forbidden wrap.** A frame is *never* split across the physical end of the arena,
so the active frame is always one contiguous `(offset, len)` slice the pump walks
linearly (and can hand off zero-copy). When an incoming frame will not fit in the
contiguous tail, the writer emits a **PAD record** that consumes the rest of the
tail and places the frame at offset 0 (the next "lap"). The writer keeps the
contiguous tail ≥ one header by eagerly emitting a PAD-to-wrap, so a PAD is always
representable and every FRAME record is physically contiguous.

### 3.1 Arena record header (inline, mirrors the wire header)

```
offset  size  field
  0      2    magic    'F' = 0x46 (FRAME)  |  'P' = 0x50 (PAD)
  1      1    version  = 1
  2      1    flags    (reserved, 0)
  3      1    —        (pad to align)   [exact layout TBD at impl]
  4      2    frame_id u16  (FRAME only; storage tag, not used to address)
  6      2    —
  8      4    len      u32  payload bytes that follow (FRAME), or tail bytes to
                            skip (PAD)
 12     len   payload  back-to-back laser_command records (FRAME only)
```

A FRAME record's payload is exactly the bytes `laser_engine` consumes — no frame
markers leak into the command stream, and the ISR never learns what a frame is.

### 3.2 Cursors & reclaim

All free-running, masked into the arena (`byte_queue` discipline):

| cursor      | advanced by      | meaning                                          |
|-------------|------------------|--------------------------------------------------|
| `write`     | TCP recv (sole)  | end of bytes written so far                      |
| `committed` | TCP recv         | end of the last **fully received** frame ("valid")|
| `playing`   | pump (at loop ⟲) | start of the frame currently rendering = reclaim tail |

- **Reclaim:** the oldest retained frame *is* the one playing; everything older is
  free. The writer may fill up to (but not into) the `playing` frame.
- **Backpressure is free:** when placing a frame would overrun `playing`, the recv
  task stops reading the socket; TCP flow control stalls the sender. No custom
  windowing, no drops.
- A single frame must fit the usable arena (`len ≤ ~96 KB − headers`); the sender
  is responsible for this and it is validated on receive.

## 4. Wire formats

### 4.1 TCP frame stream (data plane)

Frames sent back-to-back on one TCP connection. Each:

```
[u16 magic 'F'][u8 version=1][u8 flags][u16 frame_id][u16 _rsv][u32 payload_len][payload]
```

`payload` = back-to-back `laser_command` encoded records (the same
`laser_command_encode` bytes the engine queue uses). `frame_id` is a tag the device
stores (playout is relative, so it is not used to address the frame). The recv task
accumulates `payload_len` bytes, then commits. A short/torn read simply isn't committed yet (TCP guarantees
the rest arrives).

**Commit ack.** After committing a frame the device writes **one byte (`0x06`)** back
on the same TCP connection. This lets the host *wait for the commit* before sending a
NEXT tick. It is needed because the two planes are independent channels: a UDP NEXT
can otherwise overtake the still-in-flight TCP frame bytes and advance into an empty
queue (an off-by-one where the just-sent frame doesn't appear until the next send). A
local `write()` completing only means the bytes left the host — not that the device
received and committed them — so the ack, not a host-side flush, is the real signal.

### 4.2 UDP control (playout clock)

Small, fire-and-forget:

```
[u16 magic 'S' 'C'][u8 version=1][u8 type=NEXT]
```

`type=NEXT` advances the device's playout cursor **one frame** — *relative, not an
absolute id*. This is a deliberate choice: the host stays **stateless** and can
never desync from or "lose track of" the device's position; it just says *advance*.
The cost is that a lost or reordered UDP tick **lingers one extra frame** (a timing
hitch), rather than permanently dropping a frame — an acceptable trade for not
needing a position to track or a recovery path.

(An absolute `PLAY{id}` was considered: it is idempotent under loss, but it forces
the host to know the device's current id with no way to recover if they diverge.
If precise seeking is ever needed, add `PLAY{id}` *alongside* NEXT plus a feedback
channel — but that is more machinery than this path needs now.)

UDP is used — not the existing TCP connection — precisely so the tick does **not**
head-of-line-block behind buffered frame bytes or inherit TCP retransmit latency.

## 5. Frame ids: u16, storage tags

`frame_id` is **u16** (wraps at 65536) and rides the TCP frame header. Because
playout is **relative** (NEXT), the id is *not* used to address a frame — it is
just a tag the device stores (useful for logging / future absolute seeking). The
device's playout cursor consumes the arena **FIFO in commit order**, independent
of id values. Multiple NEXT ticks pending between loop boundaries advance the
cursor by that many frames (catch-up). The cursor does **not wrap**: advancing
past the last received frame empties the queue (see §6).

## 6. Playout semantics (the pump)

- The pump renders the active frame `[off+header, off+header+len)` command-by-command
  into `laser_engine_goto/laser/dwell/curve`, retrying on a full engine queue
  (the existing `while(!laser_engine_*){vTaskDelay(1);}` backpressure).
- It **drains pending NEXT ticks once at the top of each loop** and finishes the
  current loop on the old frame before switching. So frame transitions land **only
  between complete frames, never mid-frame**.
  - 0 pending → re-loop the current frame (`frame_buffer_current`).
  - N pending → consume N frames forward (`frame_buffer_advance` ×N). A burst of
    ticks catches up; a slow trickle re-loops the current frame.
  - **Past the end → empty (NO wrap).** When a NEXT advances past the last received
    frame, nothing is displayed: the pump stops feeding and the engine **underruns
    → blanks the laser** (safe). It stays empty until a new frame is received and a
    NEXT advances into it. Likewise nothing plays before the first NEXT.
- **Reclaim floor = the displayed frame + any received-but-unplayed frames ahead of
  it.** Those are never evicted, so the displayed frame's bytes are stable while the
  pump replays them without the lock. Frames already consumed (behind the cursor)
  are reclaimable; **unplayed queued frames are never dropped** — if the arena fills
  with unplayed frames the writer backpressures (TCP stalls) rather than losing one.

### 6.1 Seamless switches require closing the loop in velocity

For a glitch-free repeat and a glitch-free switch, each frame must **end where it
starts** *and* with **matching velocity** at the seam. Concretely the host emits
each frame as: blanked at center → geometry → blanked return to center, with
`v_out == v_in == 0` at the seam (zero is simplest and safe). Then "rewind to the
start of the active frame" is continuous in **position and velocity** — no jump,
no velocity step into the servo — and so is the hand-off to the next frame.

## 7. Relationship to the prior UDP streaming plan

- **Retired for this path:** the `scene_stream` reassembly window (per-command
  seq, dedup, gap-skip, mid-stream-join relock). TCP subsumes all of it for the
  data plane.
- **Kept:** the `laser_command_encode/decode` codec (one source of truth for the
  payload wire format) and the `renderer` source mux — "loop a streamed frame"
  becomes one selectable `renderer_source_t` (e.g. `src_tcp_frames`) alongside
  `src_idle`.
- **New:** the `frame_buffer` arena/FIFO component (host-testable, pure C) plus the
  TCP recv / UDP control / pump glue.

## 8. Open items / phasing

- **Memory:** 96 KB is a single static buffer competing with WiFi/lwIP/TCP in the
  original ESP32's ~300 KB DRAM. Verify heap headroom after the TCP stack is up;
  the arena size is a single config knob. (PSRAM, if present on the board variant,
  is the natural home for it.)
- **Exact header layout** (§3.1) is illustrative; pin down field order/alignment at
  implementation and host-test the codec byte-for-byte against the C struct.
- **Host side:** (a) `svg2scene` closes the loop per §6.1; (b) a TCP frame sender;
  (c) a UDP playout ticker. The sender owns playout order and repeats.
- **Ports:** pick TCP data + UDP control ports (distinct from the `udp_echo` 3333
  smoke-test and any IDN/Art-Net future sources).
```
