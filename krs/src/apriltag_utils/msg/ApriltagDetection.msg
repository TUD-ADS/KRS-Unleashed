# Index of detected tag family in detector->tag_families.
# Hardcoded but whatever
int32 family_index
# // The decoded ID of the tag
int32 id

# // How many error bits were corrected? Note: accepting large numbers of
# // corrected errors leads to greatly increased false positive rates.
# // NOTE: As of this implementation, the detector cannot detect tags with
# // a hamming distance greater than 2.
int32 hamming

# // A measure of the quality of the binary decoding process: the
# // average difference between the intensity of a data bit versus
# // the decision threshold. Higher numbers roughly indicate better
# // decodes. This is a reasonable measure of detection accuracy
# // only for very small tags-- not effective for larger tags (where
# // we could have sampled anywhere within a bit cell and still
# // gotten a good detection.)
float32 decision_margin

# // The 3x3 homography matrix describing the projection from an
# // "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
# // -1)) to pixels in the image.
float64[9] h [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]     # [3][3] flattened

# // The center of the detection in image pixel coordinates.
float64[2] c

# // The corners of the tag in image pixel coordinates. These always
# // wrap counter-clock wise around the tag.
float64[8] p # [4][2] flattened
