use kernel::errorcode;
use kernel::process;
use kernel::syscall;

pub struct SoilMoistureSyscallFilter {}

impl kernel::platform::SyscallFilter for SoilMoistureSyscallFilter {
    fn filter_syscall(
        &self,
        process: &dyn process::Process,
        syscall: &syscall::Syscall,
    ) -> Result<(), errorcode::ErrorCode> {
        let permitted = kernel::process::ShortId::Fixed(
            core::num::NonZeroU32::new(kernel::utilities::helpers::crc32_posix(
                "soil_moisture_sensor".as_bytes(),
            ))
            .unwrap(),
        );

        match syscall.driver_number() {
            Some(capsules_core::adc::DRIVER_NUM) | Some(capsules_core::gpio::DRIVER_NUM) => {
                // For GPIO and ADC, only allow "soil_moisture_sensor" to
                // access.
                if process.short_app_id() == permitted {
                    Ok(())
                } else {
                    Err(errorcode::ErrorCode::NODEVICE)
                }
            }
            _ => Ok(()),
        }
    }
}
