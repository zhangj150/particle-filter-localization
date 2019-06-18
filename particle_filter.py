from grid import *
from particle import Particle
from utils import *
from setting import *
#by Jonathan Zhang and Pravan Kalaga

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    
    for part in particles:
        # xn, yn = rotate_point(odom[0], odom[1], part.h)
        # noiseX, noiseY, noiseH = add_odometry_noise((xn, yn, odom[2]), ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        # motion_particles.append(Particle(noiseX + part.x, noiseY + part.y, part.h + noiseH))

        noiseX, noiseY, noiseH = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        xn, yn = rotate_point(noiseX, noiseY, part.h)

        motion_particles.append(Particle(xn + part.x, yn + part.y, (part.h + noiseH)%360))

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    if SPURIOUS_DETECTION_RATE == 0 and DETECTION_FAILURE_RATE == 0:
        return clean_measurement_update(particles, measured_marker_list, grid)
    else:
        return noisy_measurement_update(particles, measured_marker_list, grid)

def noisy_measurement_update(particles, measured_marker_list, grid):
    measured_particles = []

    beliefs = []
    #add random particles
    for i in range(20):
        x = random.uniform(0, grid.width)
        y = random.uniform(0, grid.height)
        h = random.uniform(0, 360)
        particles.append(Particle(x, y, h))
    for part in particles:
        partList = part.read_markers(grid)
        prob = 1
        
        if (not grid.is_in(part.x, part.y)):
            prob = 0
        else:
            for pm in partList:
                partDist = grid_distance(0,0,pm[0],pm[1])
                #tempP = 1
                bestMarkerProb = 0
                for marker in measured_marker_list:
                    markDist = grid_distance(0,0,marker[0],marker[1])
                    hdng = diff_heading_deg(marker[2],pm[2])

                    tempP = math.exp(-((((partDist - markDist)**2)/(2*MARKER_TRANS_SIGMA**2)) + ((hdng**2)/(2*MARKER_ROT_SIGMA**2))))
                    if tempP > bestMarkerProb:
                        bestMarkerProb = tempP
                prob *= bestMarkerProb + 1

        if len(partList) > len(measured_marker_list):
            diff = len(partList) - len(measured_marker_list)
            prob *= DETECTION_FAILURE_RATE ** diff
        elif len(measured_marker_list) > len(partList):
            diff = len(measured_marker_list) - len(partList)
            prob *= SPURIOUS_DETECTION_RATE ** diff
        beliefs.append(prob)

    #add random particles
    #beliefs.extend([1/(500+PARTICLE_COUNT) for i in range(500)])
    denom = sum(beliefs)
    beliefs = [i/denom for i in beliefs]


    measured_particles = random.choices(particles, beliefs, k=PARTICLE_COUNT)

    return measured_particles

def clean_measurement_update(particles, measured_marker_list, grid):
    measured_particles = []

    beliefs = []
    #add random particles
    for i in range(20):
        x = random.uniform(0, grid.width)
        y = random.uniform(0, grid.height)
        h = random.uniform(0, 360)
        particles.append(Particle(x, y, h))
    for part in particles:
        partList = part.read_markers(grid)
        prob = 1
        pairings = {}
        if (grid.is_in(part.x, part.y)==False):
            prob = 0
        else:
            for pm in partList:
                partDist = grid_distance(0,0,pm[0],pm[1])
                #tempP = 1
                bestMarkerProb = 0
                for marker in measured_marker_list:
                    markDist = grid_distance(0,0,marker[0],marker[1])
                    hdng = diff_heading_deg(marker[2],pm[2])

                    tempP = math.exp(-((((partDist - markDist)**2)/(2*MARKER_TRANS_SIGMA**2)) + ((hdng**2)/(2*MARKER_ROT_SIGMA**2))))
                    #if matched up, add it to closed list?
                    pairings[(pm, marker)] = tempP
                    # if tempP > bestMarkerProb:
                    #     bestMarkerProb = tempP

        matchings = sorted(list(pairings.values()), reverse=True)
        if len(matchings) > 0:
            for i in range(min(len(partList), len(measured_marker_list))):
                prob *= matchings[i]


        if len(partList) > len(measured_marker_list):
            diff = len(partList) - len(measured_marker_list)
            prob *= DETECTION_FAILURE_RATE ** diff
        elif len(measured_marker_list) > len(partList):
            diff = len(measured_marker_list) - len(partList)
            prob *= SPURIOUS_DETECTION_RATE ** diff

        beliefs.append(prob)
    
    #add random particles
    #beliefs.extend([1/(500+PARTICLE_COUNT) for i in range(500)])
    denom = sum(beliefs)
    beliefs = [i/denom for i in beliefs]


    measured_particles = random.choices(particles, beliefs, k=PARTICLE_COUNT)

    return measured_particles


