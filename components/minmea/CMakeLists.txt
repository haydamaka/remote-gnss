# minmea, vendored from https://github.com/kosma/minmea (commit 2dd2cd1).
# minmea.c, minmea.h and the license files are upstream, unmodified.
idf_component_register(SRCS "minmea.c"
                       INCLUDE_DIRS ".")

# newlib has no timegm(); minmea's README suggests mktime, which is equivalent
# while TZ is unset (UTC), as it is on this device.
target_compile_definitions(${COMPONENT_LIB} PRIVATE timegm=mktime)
