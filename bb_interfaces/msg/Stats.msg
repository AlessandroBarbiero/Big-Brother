# Message used to communicate statistics for a tracker

# Statistics of current state
float64       det_a
float64       loc_a
float64       motp

uint32         objects_to_detect
uint32         true_positive
uint32         false_positive
uint32         missed
# -------------------------

# Statistics of global state
float64       tot_det_a
float64       tot_loc_a
float64       tot_motp

uint32         tot_objects_to_detect
uint32         tot_true_positive
uint32         tot_false_positive
uint32         tot_missed
# -------------------------



