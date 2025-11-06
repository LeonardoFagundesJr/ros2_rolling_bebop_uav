lanzar:
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1


lanzar:
ros2 launch nero_drone full_bebop.launch.py - onlycontrol

lanzas:
ros2 run nero_drone velocity_logger 


lanzar:
ros2 run nero_drone ref_pos.py  la referencia es un vector de 8 posiciones (x,y,z,yaw,dx,dy,dz,dyaw)


lanzas para graficar el ejercicio completo tienes que estar en nero_drone/nero_drone: 
python3 graphical.py 


lanzas para graficar en tiempo real nero_drone/nero_drone:
python3 grahps.py 

