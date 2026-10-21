# One red patch (candidate red / restricted zone) found in an image.

# Centroid from image moments, in pixels (origin top-left)
float32 centroid_x
float32 centroid_y

# Offset of the centroid from the image centre, normalised to [-1, 1]
# (+x = right, +y = down in the image). Handy for steering later.
float32 offset_x
float32 offset_y

# Contour area in pixels^2 and as a fraction of the whole image
float32 area_px
float32 area_fraction

# Axis-aligned bounding box in pixels: [x, y, width, height]
int32[4] bbox_xywh

# Simplified outline of the patch, pixel coordinates (z unused)
geometry_msgs/Point32[] contour
