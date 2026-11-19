/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Sebastian Cuy */
/* sx_pass (device/sx_pass.c): the main loop's look at the SysEx frames received, over a simulated queue whose USB
 * interrupt queues a frame between any two of the loop's looks: right after a look that found none (the window
 * moden-mz59 measured on the FM-1, where the updater took and dropped an app's query) and at random. Checked: every
 * frame a reader takes reaches it exactly once and in order, the updater is given only frames no reader took (its
 * handshake still answered), and nothing is taken twice or lost.
 *   cc -std=c99 -Wall -Wextra -Werror -o sx_pass_test tests/sx_pass_test.c && ./sx_pass_test */
#include <stdint.h>
#include <stdio.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* the queue: frame k is one byte, its number; odd numbers are the updater's (a handshake), even ones a reader's */
#define QN 4096u
static uint8_t q[QN];
static uint32_t q_w, q_r, sent;
static uint32_t isr_at;                             /* 1: the interrupt queues a frame at the next empty look's end */
static uint32_t rng = 7u;
static uint32_t rnd(uint32_t n) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng % n; }
static void isr(void) { if (q_w - q_r < QN) q[q_w++ % QN] = (uint8_t)(sent++ & 0xFFu); }

static int ota_frame_get(const uint8_t **p, uint32_t *n)
{
    if (q_w == q_r) {
        if (isr_at) {                               /* the frame arrives just after this look */
            isr_at = 0;
            isr();
        }
        return 0;
    }
    *p = &q[q_r % QN];
    *n = 1u;
    return 1;
}
static void ota_frame_done(void) { q_r++; }

static uint32_t upd_seen, upd_wrong, next_read;    /* the updater's frames; a reader's frame it got; the next expected */
static uint32_t ota_service(void)                  /* ota_take: the oldest frame, done, then judged */
{
    const uint8_t *p;
    uint32_t n;
    if (!ota_frame_get(&p, &n))
        return 0;
    ota_frame_done();
    if (!(p[0] & 1u)) {
        upd_wrong++;                                /* a reader's frame, dropped by the updater */
        next_read = (p[0] + 1u) & 0xFFu;
        return 0;
    }
    upd_seen++;
    next_read = (p[0] + 1u) & 0xFFu;
    return 7u;                                      /* a handshake's decoded length */
}

#include "../device/sx_pass.c"

static uint32_t read_n, read_order;
static int reader(const uint8_t *p, uint32_t n)
{
    (void)n;
    if (p[0] & 1u)
        return 0;                                   /* not a reader's: the updater's */
    read_order += p[0] != next_read;
    next_read = (p[0] + 1u) & 0xFFu;
    read_n++;
    return 1;
}

int main(void)
{
    uint32_t i, k;

    /* the measured window, replayed: a pass finds no frame and the interrupt queues the app's query (frame 0, a
     * reader's) right after; the updater must not get it, and the next pass gives it to the reader */
    sent = 0;
    isr_at = 1;
    CHECK(sx_pass(reader) == 0, "an empty look gave the updater something");
    CHECK(q_w - q_r == 1u, "the frame that came after the look is not waiting (%u)", q_w - q_r);
    CHECK(upd_wrong == 0u, "the updater took the reader's frame that came after the look");
    CHECK(sx_pass(reader) == 0 && read_n == 1u, "the next pass did not give the reader its frame");

    /* a frame no reader takes is the updater's: the same frame, its length back for usb_cmd */
    isr();                                          /* frame 1: the updater's */
    CHECK(sx_pass(reader) == 7u && upd_seen == 1u, "the updater did not get the frame no reader took");
    CHECK(q_w == q_r, "the updater's frame is still waiting");

    /* at random: frames queued between any two looks, some right after an empty one */
    for (i = 0; i < 200000u; i++) {
        for (k = rnd(3); k; k--)
            isr();
        isr_at = rnd(4) == 0;
        (void)sx_pass(reader);
    }
    isr_at = 0;
    while (q_w != q_r)
        (void)sx_pass(reader);
    CHECK(upd_wrong == 0u, "the updater got %u of the readers' frames", upd_wrong);
    CHECK(read_order == 0u, "%u of the readers' frames out of order or lost", read_order);
    CHECK(read_n + upd_seen == sent, "frames sent %u, read %u, the updater's %u", sent, read_n, upd_seen);
    printf("%s: %u frames, %u to the readers, %u to the updater\n", fails ? "FAIL" : "ok", sent, read_n, upd_seen);
    return fails != 0;
}
