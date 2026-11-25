# C facade over XPowersLib for the AXP2101 every board in the fleet carries
# (ADR 0027 second bring-up note). Its one job at boot: enable the rails the
# radio and the GNSS module hang from — off at power-up until firmware turns
# them on — and report the battery state. XPowersLib is vendored at
# ../../third_party/XPowersLib and registers itself as an ESP-IDF component.
idf_component_register(
    SRCS "pmu.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board driver
    PRIV_REQUIRES XPowersLib
)

# XPowersLib is header-only C++ templates; exceptions/RTTI stay off like the
# radio component.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-exceptions -fno-rtti)
