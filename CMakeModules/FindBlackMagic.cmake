# - Try to find blackmagic decklink directories
# Once done this will define
#  BLACKMAGIC_FOUND - System has Blackmagic SDK
#  BLACKMAGIC_INCLUDE_DIR - The Blackmagic include directories


find_path(BLACKMAGIC_INCLUDE_DIR)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BlackMagic DEFAULT_MSG BLACKMAGIC_INCLUDE_DIR)
mark_as_advanced(BLACKMAGIC_INCLUDE_DIR)