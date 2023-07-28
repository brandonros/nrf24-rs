use bitfield::bitfield;

bitfield! {
    pub struct SetupRetr(u8);
    u8;
    pub get_auto_retransmit_count, set_auto_retransmit_count: 3, 0;
    pub get_auto_retransmit_delay, set_auto_retransmit_delay: 7, 4;
}

bitfield! {
    pub struct RfSteup(u8);
    u8;
    pub get_padding1, set_padding1: 0, 0;
    pub get_rf_pwr, set_rf_pwr: 2, 1;
    pub get_rf_dr_high, set_rf_dr_high: 3, 3;
    pub get_pll_lock, set_pll_lock: 4, 4;
    pub get_rf_dr_low, set_rf_dr_low: 5, 5;
    pub get_padding2, set_padding2: 6, 6;
    pub get_cont_wave, set_cont_wave: 7, 7;
}

bitfield! {
    pub struct DynPd(u8);
    u8;
    pub get_dpl_p0, set_dpl_p0: 0, 0;
    pub get_dpl_p1, set_dpl_p1: 1, 1;
    pub get_dpl_p2, set_dpl_p2: 2, 2;
    pub get_dpl_p3, set_dpl_p3: 3, 3;
    pub get_dpl_p4, set_dpl_p4: 4, 4;
    pub get_dpl_p5, set_dpl_p5: 5, 5;
    pub get_padding1, set_padding1: 7, 6;
}

bitfield! {
    pub struct EnAa(u8);
    u8;
    pub get_enaa_p0, set_enaa_p0: 0, 0;
    pub get_enaa_p1, set_enaa_p1: 1, 1;
    pub get_enaa_p2, set_enaa_p2: 2, 2;
    pub get_enaa_p3, set_enaa_p3: 3, 3;
    pub get_enaa_p4, set_enaa_p4: 4, 4;
    pub get_enaa_p5, set_enaa_p5: 5, 5;
    pub get_padding1, set_padding1: 7, 6;
}

bitfield! {
    pub struct EnRxAddr(u8);
    u8;
    pub get_erx_p0, set_erx_p0: 0, 0;
    pub get_erx_p1, set_erx_p1: 1, 1;
    pub get_erx_p2, set_erx_p2: 2, 2;
    pub get_erx_p3, set_erx_p3: 3, 3;
    pub get_erx_p4, set_erx_p4: 4, 4;
    pub get_erx_p5, set_erx_p5: 5, 5;
    pub get_padding1, set_padding1: 7, 6;
}

bitfield! {
    pub struct Feature(u8);
    u8;
    pub get_en_dyn_ack, set_en_dyn_ack: 0, 0;
    pub get_ack_pay, set_ack_pay: 1, 1;
    pub get_en_dpl, set_en_dpl: 2, 2;
    pub get_padding1, set_padding1: 7, 3;
}

bitfield! {
    pub struct RxPwP0(u8);
    u8;
    pub get_rx_pw_p0, set_rx_pw_p0: 5, 0;
    pub get_padding1, set_padding1: 7, 6;
}

bitfield! {
    pub struct SetupAw(u8);
    u8;
    pub get_aw, set_aw: 1, 0;
    pub get_padding1, set_padding1: 7, 2;
}

bitfield! {
    pub struct RfCh(u8);
    u8;
    pub get_rf_ch, set_rf_ch: 6, 0;
    pub get_padding1, set_padding1: 7, 7;
}

bitfield! {
    pub struct Status(u8);
    u8;
    pub get_tx_full, set_tx_full: 0, 0;
    pub get_rx_p_no, set_rx_p_no: 3, 1;
    pub get_max_rt, set_max_rt: 4, 4;
    pub get_tx_ds, set_tx_ds: 5, 5;
    pub get_rx_dr, set_rx_dr: 6, 6;
    pub get_padding1, set_padding1: 7, 7;
}

bitfield! {
    pub struct Config(u8);
    u8;
    pub get_prim_rx, set_prim_rx: 0, 0;
    pub get_pwr_up, set_pwr_up: 1, 1;
    pub get_crco, set_crco: 2, 2;
    pub get_en_crc, set_en_crc: 3, 3;
    pub get_mask_max_rt, set_mask_max_rt: 4, 4;
    pub get_mask_tx_ds, set_mask_tx_ds: 5, 5;
    pub get_mask_rx_dr, set_mask_rx_dr: 6, 6;
    pub get_padding1, set_padding1: 7, 7;
}