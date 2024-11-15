#########################
# Plotting
#########################

# Define functions here and export them, but don't define methods here.
# Methods are defined in the appropriate plotting extensions, so that they
# aren't loaded unnescesarily

# TODO add docstrings
function transform_plot! end

function splineplot end
function splinepointsplot end
function robotsplineplot end
function jointsketch end
function componentsketch end
function robotsketch end
function robotvisualize end
function annotateframes end
function opspaceplot end
function energytimeplot end

function splineplot! end
function splinepointsplot! end
function robotsplineplot! end
function jointsketch! end
function componentsketch! end
function robotsketch! end
function robotvisualize! end
function annotateframes! end
function opspaceplot! end
function energytimeplot! end

function animate_robot end
function animate_robot_kinematics! end
function link_cache_observable_to_time_observable end
function animate_robot_odesolution end


function transform_plot! end

function _update_cache_from_framenumber! end