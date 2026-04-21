set(component_srcs "lora.c" "decoder.c" "types.c")

idf_component_register(
	SRCS "${component_srcs}"
	PRIV_REQUIRES driver freertos esp_common log ra01s
	INCLUDE_DIRS "."
)
