/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* MIDI IN on the TRS jack (ported from Felucca 1.0's midi_uart.c): PH8 -> input channel 1 -> UART1
 * RX, 31250 baud, RX DMA into a 128-byte ring, polled from the TIMER5 ISR outside the render, the
 * same context as usb_poll. Channel messages and Clock / Start / Continue / Stop go into midi_in_q
 * beside USB's, as USB-MIDI packets (CIN = status >> 4) on cable 1, which tells them apart from USB's
 * (plat_midi_in's port); other system messages and SysEx are ignored (no SysEx -> UBOOT from the jack).
 * FELUCCA_MIDI_STAMP (usb.c): each message's landing time in midi_in_t24. uart_midi_peek(t), from the same fast tick as
 * usb_midi_peek (10 kHz, the ticks inside the render too), looks for bytes the DMA has written since: the slots the
 * parser has taken hold UM_MARK, so a slot that does not is a byte come in since the last peek, at about t (bytes found
 * together are spread back a byte time each, 0.32 ms at 31250 baud). A message takes its last byte's time; a byte the
 * take finds that no peek saw gets the last peek's t.
 *
 * The ring is read by content, not by the DMA count: every slot the parser has not been given holds UM_EMPTY,
 * and the reader takes bytes until it meets two in a row. HRXCNT (latched at an event) can miss a byte while the
 * byte itself is in the ring; the reader would then stay one byte behind, so a message would end only with the
 * next byte (a note-off held until the next note). Following the content cannot drift. (Felucca [Salt]
 * 0.9-salt14, ChanceTheMaker; found by keremimo on an FM-1 with an Akai MPC Sample; GPL-3.0.) */
#include "fm1_uart.h"           /* registers (felucca-bsp chip/; the host test adds it to its include path) */

#define UM_RING 128u
#define UM_EMPTY 0xFDu         /* an undefined real-time byte, never sent: a slot the parser has taken, or never had */
static volatile uint8_t um_ring[UM_RING] __attribute__((aligned(16)));
static struct {
    uint32_t rd;
    uint32_t bytes, drops, msgs;
    uint8_t st, need, got, d0, sysex;
} um;
#if FELUCCA_MIDI_STAMP
#define UM_MARK UM_EMPTY        /* (the peek looks for the slots the reader has not emptied) */
#define UM_TQ 32u               /* the bytes stamped ahead of the parser (10 ms of a full line; older: the peek's t) */
#define UM_BYTE_T (320u * FM1_TICKS_PER_US)   /* a byte at 31250 baud (10 bits) */
static uint32_t um_t[UM_TQ];    /* byte n's landing time (n counted as um.bytes) at n % UM_TQ */
static uint32_t um_ahead;       /* the bytes from um.rd on that a peek stamped */
static uint32_t um_peek_t, um_now;   /* the last peek's t; the stamp of the byte the parser has */
#endif

static int um_put(uint32_t pkt)                    /* into midi_in_q (usb.c): 0 = full */
{
    if (mi_w - mi_r >= MQ)
        return 0;
    midi_in_q[mi_w % MQ] = pkt | 0x10u;           /* cable 1: from the jack */
#if FELUCCA_MIDI_STAMP
    midi_in_t24[mi_w % MQ] = um_now;
#endif
    RING_PUBLISH();
    mi_w++;
    return 1;
}

static void uart_midi_init(void)                   /* before TIMER5 starts: PORTH read-modify-write */
{
    uint32_t i;
    for (i = 0; i < UM_RING; i++)
        um_ring[i] = UM_EMPTY;
    fm1_uart1_midi_init(um_ring, UM_RING);
}

#if FELUCCA_MIDI_STAMP
/* the bytes come in since the last peek: their slots no longer hold UM_MARK. A byte that is itself FD (line noise)
 * is passed once the one after it is in */
static void uart_midi_peek(uint32_t t)
{
    uint32_t k = 0, i;
    um_peek_t = t;
    while (um_ahead + k < UM_RING - 1u) {
        uint32_t s = (um.rd + um_ahead + k) & (UM_RING - 1u);
        if (um_ring[s] == UM_MARK && um_ring[(s + 1u) & (UM_RING - 1u)] == UM_MARK)
            break;
        k++;
    }
    for (i = 0; i < k; i++)
        um_t[(um.bytes + um_ahead + i) % UM_TQ] = t - (k - 1u - i) * UM_BYTE_T;
    um_ahead += k;
}
#endif

static uint32_t um_len(uint32_t s) { return (s & 0xE0u) == 0xC0u ? 1u : 2u; }   /* Cx Dx: 1, else 2 */

static void um_byte(uint32_t b)
{
    if (b >= 0xF8u) {                              /* real time: any time, keeps running status */
        if (b == 0xF8u || b == 0xFAu || b == 0xFBu || b == 0xFCu) {
            if (um_put(0xFu | b << 8))
                um.msgs++;
            else
                um.drops++;
        }
        return;
    }
    if (b & 0x80u) {
        um.sysex = b == 0xF0u;
        um.st = b < 0xF0u ? (uint8_t)b : 0;        /* system common / SysEx end running status */
        um.need = (uint8_t)um_len(b);
        um.got = 0;
        return;
    }
    if (um.sysex || !um.st)
        return;
    if (um.need == 2u && !um.got) {
        um.d0 = (uint8_t)b;
        um.got = 1;
        return;
    }
    {
        uint32_t d1 = um.need == 2u ? um.d0 : b, d2 = um.need == 2u ? b : 0u;
        um.got = 0;
        if (um_put((um.st >> 4) | (uint32_t)um.st << 8 | d1 << 16 | d2 << 24))
            um.msgs++;
        else
            um.drops++;
    }
}

/* the bytes the DMA has written since the last call; a received UM_EMPTY (line noise) is passed over once the byte
 * after it has landed */
static void uart_midi_take(void)
{
    uint32_t n;
    for (n = 0; n < UM_RING; n++) {
        uint32_t b = um_ring[um.rd];
        if (b == UM_EMPTY) {
            if (um_ring[(um.rd + 1u) & (UM_RING - 1u)] == UM_EMPTY)
                break;
            b = um_ring[um.rd];                    /* the DMA writes in order: this one has landed too */
        }
        um_ring[um.rd] = UM_EMPTY;
#if FELUCCA_MIDI_STAMP
        um_now = um_ahead && um_ahead <= UM_TQ ? um_t[um.bytes % UM_TQ] : um_peek_t;
        if (um_ahead)
            um_ahead--;
#endif
        um.rd = (um.rd + 1u) & (UM_RING - 1u);
        um.bytes++;
        um_byte(b);                                /* a UM_EMPTY is an unknown realtime byte: ignored */
    }
}

static void uart_midi_poll(void)                   /* TIMER5, outside the render */
{
    fm1_uart1_rx_take();                           /* acknowledges the pendings; the count is not used */
    uart_midi_take();
}
