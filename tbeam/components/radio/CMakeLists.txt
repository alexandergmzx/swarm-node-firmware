# C facade over RadioLib (vendored at ../../third_party/RadioLib, pinned to a
# specific commit past the 7.7.1 tag — see firmware/tbeam/README.md for why).
# RadioLib's own root CMakeLists.txt registers itself as an ESP-IDF component
# when it detects ESP_PLATFORM, so it needs no wrapper; this component just
# depends on it plus board (pin maps).
idf_component_register(
    SRCS "lora_radio.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board
    PRIV_REQUIRES RadioLib
)

# lora_radio.cpp is C++ (RadioLib is a C++ library); the rest of the firmware
# is C99. C++ exceptions/RTTI stay off — RadioLib needs neither and the
# smaller binary matters on a 4 MB classic-T-Beam flash.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-exceptions -fno-rtti)
