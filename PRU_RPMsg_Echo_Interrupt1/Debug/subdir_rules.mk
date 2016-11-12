################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: PRU Compiler'
	"/opt/ti/ccsv6/tools/compiler/ti-cgt-pru_2.1.3/bin/clpru" -v3 --include_path="/home/kamil/ti-processor-sdk/example-applications/pru-icss-5.0.1/include" --include_path="/opt/ti/ccsv6/tools/compiler/ti-cgt-pru_2.1.3/include" --include_path="/home/kamil/ti-processor-sdk/example-applications/pru-icss-5.0.1/include/am335x" -g --define=am3359 --define=pru0 --diag_wrap=off --display_error_number --diag_warning=225 --endian=little --hardware_mac=on --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


