add_subdirectory(ccd)

# ----- install headers
# installs the DIRECTORY and its contents to a given DESTINATION
# The directory structure is copied verbatim(word by word = exactly) to the destination
install(
    DIRECTORY ccd/
    DESTINATION  ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT ccd
    FILES_MATCHING 
    PATTERN "*.h" 
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE
)

