# 

class Grid:
    def __init__(self, file_name:str):
        self.file_name = file_name
        self.grid = []
        self.grid_size_x = 0
        self.grid_size_y = 0
        self.start = []
        self.end = []
        self.walls = []
        self.wall_relation_horz = {}
        self.wall_relation_vert = {}
    

    def open_file(self)->str:
        with open(self.file_name, 'r') as f:
            return f.read()
    

    def file_to_list(self)->list:
        data = self.open_file()
        data = data.split('\n')
        data = [list(i) for i in data]

        for i in range(len(data)):
            data[i] = [j for j in data[i] if j != ' ']

        #self.grid = self.reverse_grid(data)        
        self.grid = data
        dict_tmp = {}
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                if self.grid[i][j] == '1':
                    self.walls.append((i, j))
                # si la lettre est en majuscule et est dans l'aphabet 
                elif self.grid[i][j].isupper() and self.grid[i][j].isalpha():
                    if self.grid[i][j] not in dict_tmp:
                        dict_tmp[self.grid[i][j]] = [(0,0),(0,0)]
                    # ajouter en premiere position
                    print("start:",self.grid[i][j])
                    dict_tmp[self.grid[i][j]][0] = (i, j)
        
                elif self.grid[i][j].islower() and self.grid[i][j].isalpha():
                    if self.grid[i][j].upper() not in dict_tmp:
                        dict_tmp[self.grid[i][j].upper()] = [(0,0),(0,0)]
                    # ajouter en deuxieme position
                    print("end:",self.grid[i][j])
                    dict_tmp[self.grid[i][j].upper()][1] = (i, j)
                else:
                    pass

        for key in dict_tmp:
            self.start.append(dict_tmp[key][0])
            self.end.append(dict_tmp[key][1])
                    
            

    def reverse_grid(self, data)->list:
        data = data[::-1]
        return data
           
    def create_file_sdf(self):
        start_list = [(start[0] - 0.5, start[1] + 0.5) for start in self.start]
        #self.get_wall_relation()
        with open('grid_1.sdf', 'w') as f:
            f.write('<?xml version="1.0"?>\n')
            
            f.write('<sdf version="1.8">\n')
            f.write('  <world name="demo">\n')
            f.write('    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics">\n')
            f.write('    </plugin>\n')
            f.write('    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">\n')
            f.write('      <render_engine>ogre2</render_engine>\n')
            f.write('    </plugin>\n')
            f.write('    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster">\n')
            f.write('    </plugin>\n')

            #SUN
            f.write(' <light name="sun" type="directional">')
            f.write('<cast_shadows>true</cast_shadows>')
            f.write('<pose>0 0 10 0 0 0</pose>')
            f.write('<diffuse>0.8 0.8 0.8 1</diffuse>')
            f.write(' <specular>0.2 0.2 0.2 1</specular>\n')
            f.write('      <attenuation>\n')
            f.write('        <range>1000</range>\n')
            f.write('        <constant>0.9</constant>\n')
            f.write('        <linear>0.01</linear>\n')
            f.write('        <quadratic>0.001</quadratic>\n')
            f.write('      </attenuation>\n')
            f.write('      <direction>-0.5 0.1 -0.9</direction>\n')
            f.write('    </light>\n')
            f.write('<model name="ground_plane">\n')
            f.write('  <static>true</static>\n')
            f.write('  <link name="link">\n')
            f.write('    <collision name="collision">\n')
            f.write('      <geometry>\n')
            f.write('        <plane>\n')
            f.write('          <normal>0 0 1</normal>\n')
            f.write('          <size>100 100</size>\n')
            f.write('        </plane>\n')
            f.write('      </geometry>\n')
            f.write('    </collision>\n')
            f.write('    <visual name="visual">\n')
            f.write('      <geometry>\n')
            f.write('        <plane>\n')
            f.write('          <normal>0 0 1</normal>\n')
            f.write('          <size>100 100</size>\n')
            f.write('        </plane>\n')
            f.write('      </geometry>\n')
            f.write('      <material>\n')
            f.write('        <ambient>0.8 0.8 0.8 1</ambient>\n')
            f.write('        <diffuse>0.8 0.8 0.8 1</diffuse>\n')
            f.write('        <specular>0.8 0.8 0.8 1</specular>\n')
            f.write('      </material>\n')
            f.write('    </visual>\n')
            f.write('  </link>\n')
            f.write('</model>\n')

            for x_start,y_start in start_list:
                i = start_list.index((x_start,y_start))
                f.write('<include>\n')
                f.write('  <uri>model://crazyflie</uri>\n')
                f.write(f'  <name>crazyflie{i}</name>\n')
                f.write(f'  <pose>{x_start} {y_start} 0 0 0 0</pose>\n')
                plugin = f'''<plugin
                    filename="gz-sim-multicopter-control-system"
                    name="gz::sim::systems::MulticopterVelocityControl">
                    <robotNamespace>crazyflie</robotNamespace>
                    <commandSubTopic>crazy{i}/twist</commandSubTopic>
                    <enableSubTopic>enable</enableSubTopic>
                    <comLinkName>crazyflie/body</comLinkName>
                    <velocityGain>1.25 1.25 0.2425</velocityGain>
                    <attitudeGain>0.02 0.02 0.02</attitudeGain>
                    <angularRateGain>0.005 0.005 0.005</angularRateGain>
                    <rotorConfiguration>
                        <rotor>
                        <jointName>crazyflie/m1_joint</jointName>
                        <forceConstant>1.28192e-08</forceConstant>
                        <momentConstant>0.005964552</momentConstant>
                        <direction>1</direction>
                        </rotor>
                        <rotor>
                        <jointName>crazyflie/m2_joint</jointName>
                        <forceConstant>1.28192e-08</forceConstant>
                        <momentConstant>0.005964552</momentConstant>
                        <direction>-1</direction>
                        </rotor>
                        <rotor>
                        <jointName>crazyflie/m3_joint</jointName>
                        <forceConstant>1.28192e-08</forceConstant>
                        <momentConstant>0.005964552</momentConstant>
                        <direction>1</direction>
                        </rotor>
                        <rotor>
                        <jointName>crazyflie/m4_joint</jointName>
                        <forceConstant>1.28192e-08</forceConstant>
                        <momentConstant>0.005964552</momentConstant>
                        <direction>-1</direction>
                        </rotor>
                    </rotorConfiguration>
                </plugin>'''
                f.write(plugin)
                f.write('</include>\n')

            for wall in self.walls:
                x_mid = wall[0] - 0.5
                y_mid = wall[1] + 0.5
                key = self.walls.index(wall)
                f.write(f'<model name="wall{key}">\n')
                f.write(f'  <pose>{x_mid} {y_mid} 0.5 0 0 0</pose>\n')
                f.write('  <static>true</static>\n')
                f.write('  <link name="link">\n')
                f.write('    <visual name="visual">\n')
                f.write('      <geometry>\n')
                f.write('        <box>\n')
                f.write(f'          <size>1 1 1</size>\n')
                f.write('        </box>\n')
                f.write('      </geometry>\n')
                f.write('    </visual>\n')
                f.write('  </link>\n')
                f.write('</model>\n\n')
            f.write('  </world>\n')
            f.write('</sdf>\n')

    def create_file_txt(self):
        with open('wall_list.txt', 'w') as f:
            f.write(self.start)
            f.write(self.end)
            f.write(self.walls)

if __name__ == '__main__':
    grid = Grid('grid.txt')
    grid.file_to_list()
    print(grid.walls)
    print("start:",grid.start)
    print("end:",grid.end)
    grid.create_file_sdf()