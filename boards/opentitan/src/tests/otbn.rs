use crate::tests::run_kernel_op;
use crate::PERIPHERALS;
use core::cell::Cell;
use kernel::static_init;
use kernel::utilities::cells::TakeCell;
use kernel::{debug, ErrorCode};
use lowrisc::otbn::Client;

static MODULUS: [u8; 256] = [
    0xf9, 0x90, 0xc7, 0x94, 0xcf, 0x96, 0xd3, 0x12, 0x6f, 0x16, 0xa6, 0x50, 0x5d, 0xcb, 0xe9, 0x29,
    0x53, 0xc8, 0x44, 0x04, 0xda, 0x69, 0x2d, 0x1a, 0xc1, 0xb8, 0xa8, 0x70, 0x97, 0xb5, 0x96, 0xd8,
    0x07, 0xef, 0x2c, 0x3a, 0x66, 0x90, 0x16, 0xf9, 0x27, 0x1e, 0xf9, 0x82, 0x2b, 0x32, 0x31, 0x17,
    0x9d, 0x3b, 0x2a, 0x86, 0x0f, 0xb8, 0x2b, 0x51, 0xab, 0xd8, 0x79, 0x99, 0x1e, 0xfe, 0x94, 0x86,
    0x68, 0x12, 0xae, 0x20, 0x03, 0x07, 0xc3, 0xb3, 0x84, 0x23, 0x36, 0x91, 0xe9, 0x26, 0xc8, 0xff,
    0xc7, 0xb9, 0x8c, 0x35, 0xfb, 0xec, 0xd0, 0xb5, 0xde, 0x60, 0xb2, 0xd4, 0x64, 0x3c, 0x60, 0x94,
    0x22, 0x6f, 0xc9, 0x6c, 0x5b, 0x61, 0x13, 0x6e, 0x45, 0x26, 0x4f, 0x48, 0xc2, 0x1e, 0xe0, 0x16,
    0x58, 0x1a, 0x31, 0x69, 0x22, 0x93, 0x10, 0xa0, 0x3d, 0x26, 0xc3, 0x92, 0xa3, 0xc3, 0x40, 0xd3,
    0x33, 0x1d, 0xa3, 0x31, 0xc7, 0xe1, 0x61, 0xc5, 0xf4, 0xb5, 0x66, 0xc1, 0x31, 0xc6, 0x4f, 0xf6,
    0xa5, 0x2d, 0x1a, 0x73, 0xf4, 0x67, 0x75, 0x88, 0xf4, 0xc8, 0xc4, 0xa1, 0x3b, 0xab, 0x47, 0xc7,
    0x18, 0x5b, 0x8c, 0x47, 0x28, 0x82, 0xba, 0xad, 0x7f, 0x39, 0x80, 0x04, 0xf5, 0x77, 0x07, 0x08,
    0xe5, 0x39, 0xff, 0x8c, 0x7f, 0xfc, 0x72, 0x41, 0x1a, 0x99, 0x5a, 0x4d, 0xf7, 0xe9, 0x71, 0xf2,
    0x74, 0x6c, 0xc9, 0x11, 0xb1, 0xb8, 0x13, 0x3f, 0x9f, 0x8e, 0x08, 0x12, 0xa7, 0x5a, 0x40, 0xd0,
    0xe3, 0xaa, 0x26, 0x48, 0xb2, 0x6e, 0xa7, 0x39, 0x08, 0x06, 0x8e, 0x43, 0x74, 0xce, 0x8d, 0xfa,
    0x49, 0x10, 0xf9, 0x7b, 0xd2, 0x4a, 0xa4, 0x2f, 0x93, 0x24, 0x9d, 0x0f, 0xda, 0xd9, 0x2c, 0xd5,
    0x21, 0xc0, 0xc9, 0x61, 0xc3, 0xc6, 0x1f, 0xaf, 0xf4, 0x47, 0x1a, 0xa5, 0x2d, 0xa9, 0xc5, 0xbd,
];

static EXPECTING: [u8; 256] = [
    0x54, 0x83, 0x7c, 0xb0, 0xd9, 0x77, 0x76, 0xb5, 0xf5, 0xc8, 0x51, 0x02, 0x41, 0xab, 0xeb, 0xa6,
    0x8e, 0x01, 0x15, 0x54, 0x30, 0x9b, 0x05, 0xb6, 0xbf, 0x40, 0x3d, 0xd2, 0x95, 0x62, 0xf7, 0x42,
    0x4d, 0xf8, 0x3b, 0xd6, 0x0b, 0x9e, 0xef, 0x27, 0x2f, 0x95, 0x8e, 0x8a, 0xaf, 0x07, 0xe9, 0x54,
    0x66, 0xc0, 0xe9, 0x1c, 0xdd, 0x1b, 0xfb, 0x91, 0xe3, 0xa6, 0x83, 0x6f, 0xa4, 0x74, 0x49, 0x75,
    0x7f, 0x35, 0x8e, 0x40, 0x04, 0x72, 0xb9, 0xe2, 0x78, 0x4c, 0x4a, 0x3e, 0x37, 0xe9, 0x19, 0xe8,
    0x61, 0xf4, 0xaa, 0x7d, 0x27, 0xd1, 0x55, 0x40, 0x59, 0x5b, 0x3c, 0x88, 0x70, 0x76, 0x09, 0x49,
    0x8c, 0x3c, 0x66, 0xe1, 0x85, 0x8e, 0xe9, 0x79, 0xfe, 0x8f, 0xc0, 0xfd, 0x40, 0xbf, 0xf3, 0x87,
    0xa9, 0x45, 0xb1, 0xce, 0xb2, 0xb8, 0x4b, 0xc2, 0x60, 0xcd, 0xda, 0xe5, 0x30, 0xf3, 0xd2, 0x38,
    0xfd, 0x9d, 0x6e, 0x15, 0x5f, 0xa3, 0x24, 0x22, 0x90, 0x08, 0x09, 0x2b, 0x2d, 0x6e, 0x15, 0xe0,
    0x97, 0x31, 0x1f, 0x85, 0x47, 0x72, 0x69, 0xf9, 0xd2, 0x5a, 0xcc, 0xe4, 0x9d, 0x17, 0xf2, 0x81,
    0x73, 0x8c, 0x40, 0x61, 0x56, 0x6f, 0xbf, 0xd0, 0xa5, 0x20, 0xed, 0x37, 0x22, 0x5a, 0xab, 0xb6,
    0x8e, 0x12, 0x87, 0x1b, 0xcd, 0x34, 0xda, 0x79, 0x0d, 0x35, 0x7c, 0xa4, 0xd1, 0xfa, 0x44, 0x09,
    0xb9, 0xf0, 0x0b, 0xb2, 0xfb, 0xd3, 0xf1, 0xfd, 0xd8, 0x2f, 0x30, 0x15, 0xe2, 0x75, 0x18, 0x90,
    0x3b, 0x33, 0xc5, 0x4a, 0x3d, 0x19, 0xd1, 0xb9, 0x35, 0x59, 0x2d, 0x2a, 0x0a, 0x51, 0xfe, 0xad,
    0x03, 0xcd, 0x05, 0x8c, 0xb6, 0xeb, 0x5f, 0x66, 0xb9, 0x40, 0x1e, 0xd0, 0xce, 0xa5, 0xe1, 0x8e,
    0x47, 0xb7, 0xb7, 0x55, 0x06, 0x92, 0xe5, 0x6f, 0xc9, 0x92, 0xc7, 0x80, 0x26, 0x2d, 0x3f, 0x2d,
];

struct OtbnTestCallback {
    op_done: Cell<bool>,
    output_buf: TakeCell<'static, [u8]>,
}

unsafe impl Sync for OtbnTestCallback {}

impl<'a> OtbnTestCallback {
    fn new(output_buf: &'static mut [u8]) -> Self {
        OtbnTestCallback {
            op_done: Cell::new(false),
            output_buf: TakeCell::new(output_buf),
        }
    }

    fn reset(&self) {
        self.op_done.set(false);
    }
}

impl<'a> Client<'a> for OtbnTestCallback {
    fn op_done(&'a self, result: Result<(), ErrorCode>, output: &'static mut [u8]) {
        self.op_done.set(true);
        assert_eq!(result, Ok(()));
        assert_eq!(output[0..256], EXPECTING);
        // Keep copy of output
        self.output_buf.replace(output);
    }
}

/// These symbols are defined in the linker script.
extern "C" {
    /// Beginning of the ROM region containing app images.
    static _sapps: u8;
    /// End of the ROM region containing app images.
    static _eapps: u8;
}

/// Static init an OtbnTestCallback, with
/// a respective buffer allocated.
/// Note: static_init!() returns an &'static mut reference.
unsafe fn static_init_test_cb() -> &'static OtbnTestCallback {
    let output_buf = static_init!([u8; 1024], [0; 1024]);

    static_init!(OtbnTestCallback, OtbnTestCallback::new(output_buf))
}

#[test_case]
fn otbn_run_rsa_binary() {
    let perf = unsafe { PERIPHERALS.unwrap() };
    let otbn = &perf.otbn;
    let cb = unsafe { static_init_test_cb() };
    let output = cb.output_buf.take().unwrap();
    let mut temp: [u8; 4] = [0; 4];

    debug!("check otbn run binary...");

    if let Ok((imem_start, imem_length, dmem_start, dmem_length)) = unsafe {
        crate::otbn::find_app(
            "otbn-rsa",
            core::slice::from_raw_parts(
                &_sapps as *const u8,
                &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
            ),
        )
    } {
        let slice = unsafe { core::slice::from_raw_parts(imem_start as *const u8, imem_length) };

        run_kernel_op(100);

        cb.reset();
        otbn.set_client(cb);
        assert_eq!(otbn.load_binary(slice), Ok(()));

        run_kernel_op(1000);

        let slice = unsafe { core::slice::from_raw_parts(dmem_start as *const u8, dmem_length) };

        cb.reset();
        assert_eq!(otbn.load_data(0, slice), Ok(()));
        run_kernel_op(1000);

        // Set the RSA mode to encryption
        // The address is the offset of `mode` in the RSA elf
        temp[0] = 1;
        assert_eq!(otbn.load_data(0, &temp), Ok(()));

        run_kernel_op(1000);

        // Set the RSA length
        // The address is the offset of `n_limbs` in the RSA elf
        temp[0] = (MODULUS.len() / 32) as u8;
        assert_eq!(otbn.load_data(4, &temp), Ok(()));

        run_kernel_op(1000);

        // Set the RSA modulus
        // The address is the offset of `modulus` in the RSA elf
        assert_eq!(otbn.load_data(0x420, &MODULUS), Ok(()));
        run_kernel_op(1000);

        // Set the data in
        // The address is the offset of `in` in the RSA elf
        let mut source: [u8; 256] = [0; 256];
        source[0..14].copy_from_slice(b"OTBN is great!");

        assert_eq!(otbn.load_data(0x820, &source), Ok(()));
        run_kernel_op(1000);

        cb.reset();
        // The address is the offset of `out` in the RSA elf
        assert_eq!(otbn.run(0x288, output), Ok(()));

        run_kernel_op(10000);

        #[cfg(feature = "hardware_tests")]
        assert_eq!(cb.op_done.get(), true);

        debug!("    [ok]");
        run_kernel_op(100);
    } else {
        debug!("    [FAIL] No OTBN binary");
        run_kernel_op(100);
    }
}
