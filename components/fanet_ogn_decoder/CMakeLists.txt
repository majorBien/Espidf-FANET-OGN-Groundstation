set(component_srcs "rf.c" "multi_decoder.c" "fanet_types.c" "ogn_types.c")

idf_component_register(
	SRCS "${component_srcs}"
	PRIV_REQUIRES driver freertos esp_common log ra01s
	INCLUDE_DIRS "."
)
