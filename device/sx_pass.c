/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Sebastian Cuy */
/* The main loop's look at the SysEx frames received (felucca-bsp's chip/usb.c queue: ota_frame_get, ota_frame_done):
 * the oldest is offered to take (the platform's readers, then the app's: 1 = taken), and the updater (ota_service,
 * which gets the oldest frame itself) looks only at a frame none of them took: the same frame, still the oldest.
 *
 * The updater used to run on every pass that took nothing, also on a pass that found no frame. A frame the USB
 * interrupt queued between this look and the updater's own (ota_take's ota_frame_get) went to the updater, which drops
 * whatever is not in its framing: on the FM-1 about 1 in 900 of an app's queries (moden's build id, F0 7D 'M' 'O' 'D'
 * 0F 00 F7, one open port, 55 ms apart) was never answered, each one counted in ota_take (moden-mz59, 2026-11-19). A
 * frame that arrives after the look now waits for the next pass.
 *
 * Part of main_fm1.c (the app's main loop and safe mode's); on its own for tests/sx_pass_test.c, which interrupts it
 * with a frame between any two looks. Returns ota_service's decoded length for usb_cmd (0: no frame for the updater). */
static uint32_t sx_pass(int (*take)(const uint8_t *p, uint32_t n))
{
    const uint8_t *p;
    uint32_t n;
    if (!ota_frame_get(&p, &n))
        return 0;                                   /* none: the updater does not look either */
    if (take(p, n)) {
        ota_frame_done();
        return 0;
    }
    return ota_service();                           /* the frame none took (the oldest until ota_frame_done) */
}
