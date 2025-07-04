import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import numpy as np
import random

# Define the singularity point (center of the bed)
SINGULARITY_POINT = (0, 0)

#refer to photo in instructable
MAX_SPEED = 1500  # Max speed away from the singularity
MIN_SPEED = 100   # Min speed 
CENTER_MIN=20 #speed in center circle
MAX_DISTANCE = 30 # this radius along with MAX & MIN_SPEED are used to compute the speed at any radius

SPLIT_RADIUS=2 #any line passing wthin this radius split into 3 segments
#the middle segment speed to be changed to CENTER_MIN as there will be massive angle moves
SPLIT_START_RADIUS=2.5 #the radius that determines the points where the line splits actually occurs
#want to be slightly bigger than SPLIT_RADIUS so that all the lines crossing through SPLIT_RADIUS will have a proper slow segment
#without a slightly larger radius, a line just barely passing (practically tangent) within SPLIT_RADIUS would
#be split into two lines and like a point as that's all that would fit within that sliver of the circle
CENTER_R=0.01 # Any line passing within this radius is going through the dead center



def point_line_distance(start, end):
    """
    Calculate the minimum distance between a line segment (start, end) and a point in 2D.
    
    Args:
    start (tuple or list): Coordinates of the start point of the line (x1, y1).
    end (tuple or list): Coordinates of the end point of the line (x2, y2).
    point (tuple or list): Coordinates of the point (px, py).
    
    Returns:
    float: The minimum distance between the point and the line segment.
    """

    # Convert points to numpy arrays for vector math
    start = np.array(start)
    end = np.array(end)
    point = np.array(SINGULARITY_POINT)

    # Vector from start to end (line direction)
    line_vec = end - start

    # Vector from start to the point
    point_vec = point - start

    # Project point_vec onto line_vec to find the closest point on the line
    line_len = np.dot(line_vec, line_vec)  # Squared length of the line
    if line_len == 0:
        # Start and end points are the same
        return np.linalg.norm(point_vec)  # Distance from point to this single point

    projection = np.dot(point_vec, line_vec) / line_len

    # Clamp projection to the range [0, 1] so the closest point is within the segment
    projection = max(0, min(1, projection))

    # Find the closest point on the line segment
    closest_point = start + projection * line_vec

    # Return the distance between the point and the closest point on the line
    return np.linalg.norm(closest_point - point)


def distance_to_singularity(x, y):
    """Calculate the distance from a point to the singularity."""
    return math.sqrt((x - SINGULARITY_POINT[0]) ** 2 + (y - SINGULARITY_POINT[1]) ** 2)


def adjust_speed(distance):
    """Adjust the speed based on the distance to the singularity."""
    # Speed scales linearly between MIN_SPEED and MAX_SPEED based on distance
    new_speed= round(distance * (MAX_SPEED - MIN_SPEED)/(MAX_DISTANCE) +MIN_SPEED,1)
    if new_speed>MAX_SPEED:
        return MAX_SPEED
    else:
        return new_speed
    

def polar_coordinates(x, y):
    """Convert cartesian (x, y) to polar (r, theta)."""
    r = distance_to_singularity(x, y)
    theta = math.atan2(y, x)  # Angle in radians
    return r, theta





def plot_gcode_movements(movements, title="G-code Movements", random_colors = False,color_by_speed = False):
    """Plot the G-code movements as lines on a polar graph, with optional color gradients by speed."""
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_rlim(0, 25) #change dimensions of plot
    first_line=True
    
    cmap = matplotlib.colormaps.get_cmap('coolwarm')  # Colormap for speed visualization

    for move in movements:
        (x_start, y_start), (x_end, y_end), speed = move
        r_start, theta_start = polar_coordinates(float(x_start), float(y_start))
        r_end, theta_end = polar_coordinates(float(x_end), float(y_end))
        
        # Determine the color based on speed if color_by_speed is True
        if random_colors:
            color = (random.random(),random.random(),random.random())
        elif color_by_speed:
            norm_speed = (speed - CENTER_MIN) / (MAX_SPEED - CENTER_MIN)
            color = cmap(norm_speed)
        else:
            color = 'b'  # Default color if not using speed gradient
        if(first_line):
            ax.plot([theta_start, theta_end], [r_start, r_end], color='r', label=f"Move from ({x_start}, {y_start}) to ({x_end}, {y_end})")
            first_line = False
        else:
            ax.plot([theta_start, theta_end], [r_start, r_end], color=color, label=f"Move from ({x_start}, {y_start}) to ({x_end}, {y_end})")


    ax.set_title(title)
    plt.show()



    
def split_movement_through_singularity(start, end):
    """If a movement passes through the singularity, split it into 3 lines."""


    if(point_line_distance(start,end)<SPLIT_RADIUS): 
        return True
  
    else:
        #print("start")
        #print(start)
       #print("end")
        #print(end)
        return False
 


def do_plotting(original_movements, modified_movements):
    plot_gcode_movements(original_movements, title="Original G-code Movements", color_by_speed=True)

    # Plot the modified movements with random colors
    plot_gcode_movements(modified_movements, title="Modified G-code Movements (Speed Color Temperature)", color_by_speed=True)
    plot_gcode_movements(modified_movements, title="Modified G-code Movements (Random Colors)", random_colors=True)



#function that calculates the intersection points between a line between x1,y1 and x2,y2 and a circle of radius r
def line_circle_intersection(x1, y1, x2, y2, r):
    # Calculate the coefficients of the line equation in the form: Ax + By = C
    dx = x2 - x1
    dy = y2 - y1

    # Quadratic coefficients (a, b, c) for line-circle intersection
    a = dx**2 + dy**2
    b = 2 * (x1 * dx + y1 * dy)
    c = x1**2 + y1**2 - r**2

    # Calculate the discriminant
    discriminant = b**2 - 4 * a * c

    if discriminant < 0:
        # No intersection
        return []

    elif discriminant == 0:
        # One intersection (tangent line)
        t = -b / (2 * a)
        if 0 <= t <= 1:
            return [[(round(x1 + t * dx, 3), round(y1 + t * dy, 3))]]
        else:
            return []

    else:
        # Two intersections
        sqrt_discriminant = math.sqrt(discriminant)

        # First solution
        t1 = (-b + sqrt_discriminant) / (2 * a)
        intersection_points = []
        if 0 <= t1 <= 1:
            intersection_points.append((x1 + t1 * dx, y1 + t1 * dy))

        # Second solution
        t2 = (-b - sqrt_discriminant) / (2 * a)
        if 0 <= t2 <= 1:
            intersection_points.append((x1 + t2 * dx, y1 + t2 * dy))

        # Sort the intersection points based on their distance to (x1, y1)
        intersection_points.sort(key=lambda point: math.sqrt((point[0] - x1) ** 2 + (point[1] - y1) ** 2))

        # Round the points to one decimal place
        return [(round(point[0], 3), round(point[1], 3)) for point in intersection_points]


#function that writes the modified gcode to the output file
#parts: holds the most recent line of gcode
#modified_movements: holds a list of all the new gcode

def write_new_gcode(parts, modified_movements, output, E=None):
    (x_start, y_start), (x_end, y_end), speed = modified_movements[-1]
    f_in_parts=False
    for part in parts: #check if old line of gcode had an 'F' value
        if part.startswith('F'):
            f_in_parts = True

    new_gcode=""
    if f_in_parts: #if 'F' modify it
        for part in parts:
            if part.startswith('X'):
                new_gcode+= 'X'+str(x_end)+ ' '
            elif part.startswith('Y'):
                new_gcode+= 'Y'+str(y_end)+ ' '
            elif part.startswith('F'):
                new_gcode+= 'F'+str(speed)+ ' '
            elif part.startswith('E') and E != None:
                new_gcode+= 'E'+str(E)+ ' '
            else:
                new_gcode+=part+ ' '
        new_gcode+="\n"
        
        
    else: #if no 'F,' add an 'F' part
        for part in parts:
            if part.startswith('X'):
                new_gcode+= 'X'+str(x_end)+ ' '
            elif part.startswith('Y'):
                new_gcode+= 'Y'+str(y_end)+ ' '
            elif part.startswith('E') and E!= None:
                new_gcode+= 'E'+str(E)+ ' '
            elif not part.startswith(';'):
                new_gcode+=part+ ' '
        new_gcode+="F"+str(speed)
        for part in parts:
            if part.startswith(';'):
                new_gcode+=part+ ' '
        new_gcode+="\n"
         
    output.write(new_gcode)




#in splitting an old gcode line into multiple new ones, you have to figure out how 
# much filament each new line needs
#the function figures out the fraction of the full line extrusion value that is needed for a particular split segment
# e is the total extrusion value for the whole line

def fraction_extruded(e, total_line_length, part_line_length):
    if total_line_length>0 and e != None:
        #print(round(part_line_length/total_line_length,5)*100)
        return e*part_line_length/total_line_length



def line_length(x1,y1,x2,y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx**2 + dy**2)



#function that handles the 180° rotation when a line is going through the origin
#the hotend is coming into the origin at an angle
#the function computes the incomming angle and then the outgoing angle
#these angles are used to create points for the hotend to move to to rotate it 180
#its the semicircle on the plots
def rotate_about_center(output,modified_movements):
    rotation_speed=100
    rotation_radius=5
    print(modified_movements[-1])
    (x_start, y_start), (x_end, y_end), speed = modified_movements[-1]
    # Calculate the starting angle of the arc based on the start point
    start_angle = np.arctan2(y_start,x_start)
    print(x_start)
    print("start")
    print("start_angle")
    print(math.degrees(start_angle))
    
    # Arc spans 180 degrees, so the end angle is start_angle + 180 degrees (or pi radians)
    end_angle = start_angle - np.pi
    #print(math.degrees(end_angle))
    num_angles=10
  
    angles = np.linspace(start_angle, end_angle, num=num_angles)
    points=[]
    #print(points)

    for angle in angles:
        points.append((rotation_radius * np.cos(angle), rotation_radius * np.sin(angle)))
    
#retract the filament 
    output.write("G91\n")
    output.write("G0 E-1 F800\n")
    output.write("G90\n")



    modified_movements.append(((x_end,y_end), (round(points[0][0],3),round(points[0][1],3)),rotation_speed))
    for i in range(num_angles-1):
        modified_movements.append(((round(points[i][0],3),round(points[i][1],3)),(round(points[i+1][0],3),round(points[i+1][1],3)),rotation_speed))

#create gcode for rotation
    for point in points:
        gcode="G0 X"
        gcode+=str(round(point[0],3))+" Y"
        gcode+=str(round(point[1],3))+" F"
        gcode+=str(rotation_speed)+"\n"
        output.write(gcode)
        #output.write("G4 P10000\n")
        

        
    
    

#move back into position
    
    x_new_start=0.002 * np.cos(end_angle)
    y_new_start=0.002 * np.sin(end_angle)
    modified_movements.append(((points[-1]),(x_new_start,y_new_start),rotation_speed))
    gcode="G0 X"
    gcode+=str(round(x_new_start,3))+" Y"
    gcode+=str(round(y_new_start,3))+" F"
    gcode+=str(rotation_speed)+"\n"
    # modified_movements.append(((round(points[-1][0],0),round(points[-1][1],0)),(0,0),100))
    # gcode="G0 X0 Y0 F100\n"
    output.write(gcode)

#put filament back into position
    output.write("G91\n")
    output.write("G0 E0.8 F800\n")
    output.write("G90\n")


    return (x_new_start,y_new_start)

    
 #############     
 #############     
 #############     
 ############# 
    
 ############# Main function  

def process_gcode_and_plot(input_file):
    

    original_movements = []
    modified_movements = []
    prev_e=0


    first_G=True #make sure first G line is added to the output file

    # Open the input G-code file for reading
    with open(input_file+'.gcode', 'r') as f:
        lines = f.readlines()

    # Open the output G-code file for writing
    with open(input_file+'_processed.gcode', 'w') as output:



        i =0 #keep track of lines

        while i< len(lines):
            #go searching for first line of G movement
            while((not lines[i].startswith("G1") and not lines[i].startswith("G0")) or 'X' not in lines[i] or 'Y' not in lines[i]):
                #print(lines[i])
                output.write(lines[i])
                i+=1
                #put at end
                if i == len(lines):
                    do_plotting(original_movements, modified_movements)
                    return
                if lines[i].startswith("G92 E0"):
                    prev_e=0
            
            #found first G line so process it
            if first_G:
                output.write(lines[i])
                first_G=False

            parts = lines[i].split()
            x, y = None, None

            for part in parts:
                if part.startswith('X'):
                    #print(part)
                    x = float(part[1:])
                elif part.startswith('Y'):
                    #print(part)
                    y = float(part[1:])
                elif part.startswith('E'):
                    prev_e = float(part[1:])
                
            
            i+=1 #increment line number

# go searching for next G line 
            while((not lines[i].startswith("G1") and not lines[i].startswith("G0")) or 'X' not in lines[i] or 'Y' not in lines[i]):
                output.write(lines[i])
                i+=1
                #put at end
                if i == len(lines):
                    do_plotting(original_movements, modified_movements)
                    return
                if lines[i].startswith("G92 E0"):
                    prev_e=0
                    
             #found G line so process it   
            parts = lines[i].split()
            next_x, next_y, next_e = None, None, None

            for part in parts:
                if part.startswith('X'):
                    next_x = float(part[1:])
                elif part.startswith('Y'):
                    next_y = float(part[1:])
                elif part.startswith('E'):
                    next_e = float(part[1:])


            # Store original movement for plotting
            original_movements.append(((x, y), (next_x, next_y), 0))




            ################################################
            #MODIFIES THE GCODE 

            #check if movement is super near singularity 
            split_moves = split_movement_through_singularity((x, y), (next_x, next_y))
            #split_moves = False

            if split_moves:
                # 4 scenarios: line goes through center circle
                # line goes into center circle
                # line leaves center circle
                #line starts and ends in center circle
                #output.write(";split through middle\n")

                #find intersection point(s) between line and circle around origin 
                intersection_points=line_circle_intersection(x,y,next_x, next_y, SPLIT_START_RADIUS)

                if(distance_to_singularity(x,y)>SPLIT_START_RADIUS and distance_to_singularity(next_x,next_y)>SPLIT_START_RADIUS):
                    #goes through circle
                    #need to split into three segments- line to circle, line in circle, line out of circle

                    #segment going to circle
                    print("Through center circle")
                    new_speed = adjust_speed(SPLIT_START_RADIUS)

                    modified_movements.append(((x,y),(intersection_points[0][0],intersection_points[0][1]), new_speed))
                    #print(modified_movements[-1])
                    if next_e != None:
                        fractional_extruded=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(x,y,intersection_points[0][0],intersection_points[0][1]))
                       #print(prev_e+fractional_extruded)
                        output.write(";going into circle\n")
                        write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        
                    else:
                        write_new_gcode(parts, modified_movements, output)



                    #segment through circle at min speed
                            #if goes through dead center, split into move to center, rotate 180 degrees, move away from center
                    
                    if point_line_distance(intersection_points[0],intersection_points[1])<=CENTER_R: #split it
                        print("dead center move\n")
                        #move to dead center
                        output.write(";to center\n")
                        start_angle = np.arctan2(intersection_points[0][1], intersection_points[0][0])


                        x_new_end=0.002 * np.cos(start_angle) #actually moving to very very close to dead center
                        y_new_end=0.002 * np.sin(start_angle)
                        #x_new_end=0
                        #y_new_end=0

                        
                        modified_movements.append(((intersection_points[0][0],intersection_points[0][1]), (x_new_end,y_new_end),MIN_SPEED))
                        
                        #get fraction extruded just through whole circle
                        fraction_extruded_center_circle=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[1][0],intersection_points[1][1],intersection_points[0][0],intersection_points[0][1]))
                        if next_e !=None:
                            #use the amount extruded through whole circle to find amount extruded for the line just to the center
                            fractional_extruded+=fraction_extruded(fraction_extruded_center_circle, line_length(intersection_points[1][0],intersection_points[1][1],intersection_points[0][0],intersection_points[0][1]), line_length(intersection_points[0][0],intersection_points[0][1],x_new_end,y_new_end))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                        ## rotate about center
                       #output.write("G4 P10000\n")
                        output.write("; rotate about center\n")
                    
                        new_start=rotate_about_center(output,modified_movements)

                        #move away from center
                        output.write(";leaving center\n")
                        modified_movements.append(((new_start[0],new_start[1]),(intersection_points[1]),MIN_SPEED))
                        if next_e !=None:
                                #use the amount extruded through whole circle to find amount extruded for the line away from the cetner
                            fractional_extruded+=fraction_extruded(fraction_extruded_center_circle, line_length(intersection_points[1][0],intersection_points[1][1],intersection_points[0][0],intersection_points[0][1]), line_length(intersection_points[1][0],intersection_points[1][1],new_start[0],new_start[1]))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                    

                        

                    else:
                        output.write(";going through circle but not center\n")
                        modified_movements.append((intersection_points[0],intersection_points[1], CENTER_MIN))

                        if next_e !=None:
                            fractional_extruded+=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[1][0],intersection_points[1][1],intersection_points[0][0],intersection_points[0][1]))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

#segment leaving circle
                    output.write(";leaving circle\n")
                    modified_movements.append((intersection_points[1], (next_x, next_y),new_speed))

                    if next_e !=None:
                        fractional_extruded+=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[1][0],intersection_points[1][1],next_x,next_y))
                        write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                    else:
                        write_new_gcode(parts, modified_movements, output)


##########################

                elif len(intersection_points)==1 and distance_to_singularity(x,y)>SPLIT_START_RADIUS:
                    #line going into singularity circle
                    new_speed = adjust_speed(SPLIT_START_RADIUS)
                    print("Line into Circle")

                    modified_movements.append(((x,y),intersection_points[0], new_speed))
                    if next_e !=None:
                        fractional_extruded=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(x,y,intersection_points[0][0],intersection_points[0][1]))
                        write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        
                    else:
                        write_new_gcode(parts, modified_movements, output)

                    #segment through circle at min speed
                                #if goes through dead center, split into move to center, rotation, move away from center
                    
                    if point_line_distance(intersection_points[0],(next_x,next_y))<=CENTER_R: #split it
                        #move to dead center
                        start_angle = np.arctan2(intersection_points[0][1], intersection_points[0][0])


                        x_new_end=0.002 * np.cos(start_angle)
                        y_new_end=0.002 * np.sin(start_angle)
                        #x_new_end=0
                        #y_new_end=0
                        modified_movements.append((intersection_points[0], (x_new_end,y_new_end),MIN_SPEED))
                        
                        #get fraction extruded just through whole circle
                        if next_e !=None:
                            fraction_extruded_center_circle=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[0][0],intersection_points[0][1],next_x,next_y))

                            #use the amount extruded through whole circle to find just the amount extruded to (or away) the center
                            fractional_extruded+=fraction_extruded(fraction_extruded_center_circle, line_length(intersection_points[0][0],intersection_points[0][1],next_x,next_y), line_length(intersection_points[0][0],intersection_points[0][1],x_new_end,y_new_end))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                        ## rotate about center
                    
                        new_start=rotate_about_center(output,modified_movements)

                        #move away from center
                        modified_movements.append(((new_start[0],new_start[1]),(next_x,next_y),MIN_SPEED))
                        if next_e !=None:
                                #use the amount extruded through whole circle to find just the amount extruded to (or away) the center
                            fractional_extruded+=fraction_extruded(fraction_extruded_center_circle, line_length(intersection_points[0][0],intersection_points[0][1],next_x,next_y), line_length(next_x,next_y,new_start[0],new_start[1]))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                    

                        

                    else:
                        modified_movements.append((intersection_points[0],(next_x,next_y), CENTER_MIN))
                        if next_e !=None:
                            fractional_extruded+=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[0][0],intersection_points[0][1],next_x,next_y))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

########################

                elif len(intersection_points)==1 and distance_to_singularity(next_x,next_y)>SPLIT_START_RADIUS:
                    #line leaving singularity circle
                    print("Line Leaving Circle")
                 

                    new_speed = adjust_speed(SPLIT_START_RADIUS)

                    #       #segment through circle at min speed
                                #if goes through dead center, split into move to center, rotation, move away from center
                    
                    if point_line_distance(intersection_points[0],(x,y))<=CENTER_R: #split it
                        #move to dead center
                        start_angle = np.arctan2(x, y)
 
                        x_new_end=0.002 * np.cos(start_angle)
                        y_new_end=0.002 * np.sin(start_angle)
                        #x_new_end=0
                        #y_new_end=0
                        
                        modified_movements.append(((x,y), (x_new_end,y_new_end),MIN_SPEED))
                        
                        #get fraction extruded just through whole circle
                        fraction_extruded_center_circle=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[0][0],intersection_points[0][1],x,y))
                        if next_e !=None:
                            #use the amount extruded through whole circle to find just the amount extruded to (or away) the center
                            fractional_extruded=fraction_extruded(fraction_extruded_center_circle, line_length(intersection_points[0][0],intersection_points[0][1],x,y), line_length(x,y,x_new_end,y_new_end))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                        ## rotate about center
                    
                        new_start=rotate_about_center(output,modified_movements)

                        #move away from center
                        modified_movements.append(((new_start[0],new_start[1]),intersection_points[0],MIN_SPEED))
                        if next_e !=None:
                                #use the amount extruded through whole circle to find just the amount extruded to (or away) the center
                            fractional_extruded+=fraction_extruded(fraction_extruded_center_circle, line_length(intersection_points[0][0],intersection_points[0][1],x,y), line_length(intersection_points[0][0],intersection_points[0][1],new_start[0],new_start[1]))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                    

                        

                    else:
                        modified_movements.append(((x,y),intersection_points[0], CENTER_MIN))
                        if next_e !=None:
                            fractional_extruded=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[0][0],intersection_points[0][1],x,y))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                            
                        else:
                            write_new_gcode(parts, modified_movements, output)
#segment leaving circle
                    modified_movements.append((intersection_points[0],(next_x,next_y), new_speed))
                    if next_e !=None:
                        fractional_extruded+=fraction_extruded(next_e-prev_e, line_length(x,y,next_x,next_y), line_length(intersection_points[0][0],intersection_points[0][1],next_x,next_y))
                        write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                    else:
                        write_new_gcode(parts, modified_movements, output)



                elif len(intersection_points)==0:
                    #line segment entirely in the circle 
                    print("Line Entirely in Circle")
                    if point_line_distance((next_x,next_y),(x,y))<=CENTER_R: #split it
                        #move to dead center
                        start_angle = np.arctan2(x, y)
 
                        # x_new_end=0.002 * np.cos(start_angle)
                        # y_new_end=0.002 * np.sin(start_angle)
                        x_new_end=0
                        y_new_end=0

                        modified_movements.append(((x,y), (x_new_end,y_new_end),MIN_SPEED))
                        
                        if next_e !=None:
                            
                            fractional_extruded=fraction_extruded(next_e-prev_e, line_length(next_x,next_y,x,y), line_length(x,y,0,0))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                        ## rotate about center
                    
                        new_start=rotate_about_center(output,modified_movements)

                        #move away from center
                        modified_movements.append(((new_start[0],new_start[1]),(next_x,next_y),MIN_SPEED))
                        if next_e !=None:
                                
                            fractional_extruded+=fraction_extruded(next_e-prev_e, line_length(next_x,next_y,x,y), line_length(next_x,next_y,new_start[0],new_start[1]))
                            write_new_gcode(parts, modified_movements, output, round(prev_e+fractional_extruded,5))
                        else:
                            write_new_gcode(parts, modified_movements, output)

                    

                        

                    else:

                        modified_movements.append(((x,y),(next_x, next_y), CENTER_MIN))
                        write_new_gcode(parts, modified_movements, output)

                

            else:
            # Adjust speed and write to output G-code
                            # Modify based on proximity to singularity
                min_distance_singularity = point_line_distance((x,y),(next_x, next_y))

                new_speed = adjust_speed(min_distance_singularity)
   
                modified_movements.append(((x, y), (next_x, next_y), new_speed))
                write_new_gcode(parts, modified_movements, output)
                


    #Plot the original movements
    do_plotting(original_movements, modified_movements)



##########call the function
input_gcode = 'CFFFP_Test+Print+v11.gcode'  # Replace with your actual G-code file
process_gcode_and_plot(input_gcode)     













