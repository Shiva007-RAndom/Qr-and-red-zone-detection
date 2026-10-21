# Result of running red-patch detection on one image.
# The header is copied from the incoming image so results stay
# time-synchronised with the source frame.
std_msgs/Header header

# Image size the pixel values refer to
uint32 image_width
uint32 image_height

# True if at least one red patch was found
bool detected

# Number of patches kept (area >= min_area)
uint16 num_patches

# Sum of the patch areas as a fraction of the image
float32 total_area_fraction

# Patches ordered by descending area
RedPatch[] patches

# Wall-clock processing time for this frame, in milliseconds
float32 processing_time_ms
