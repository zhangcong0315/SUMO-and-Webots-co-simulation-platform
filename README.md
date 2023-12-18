The Co-Simulation Platform for Driving Simulators is a pioneering project aimed at addressing the limitations of existing commercial driving simulators. Traditionally, simulators such as STISIM, VirtualDriver, and DriveSafety have been hindered by constraints like predefined road geometry, fixed areas, and the inability to accurately capture realistic vehicle behaviors. Motivated by these limitations, our team embarked on developing a cutting-edge co-simulation platform that seamlessly integrates two core components: Simulation of Urban Mobility (SUMO) and Webots. This platform goes beyond conventional driving simulators, offering an unprecedented level of flexibility and realism, with the potential to extend functionalities to include autonomous driving features like perception sensors.

The driving simulator, based on the co-simulation platform, takes advantage of SUMO's microscopic traffic simulation capabilities widely utilized in transportation research. SUMO generates the road network based on Open Street Map (OSM) data and simulates background vehicle traffic. Webots, a versatile robotic simulator, serves as a connector to the driving simulator, providing comprehensive modeling and simulation capabilities for 3D roadway environments and vehicle dynamics. It can also model autonomous vehicle functionalities, such as attaching perception sensors to simulated vehicles.

The co-simulation platform's structure, illustrated in Figure 1, showcases the seamless integration of SUMO and Webots. Real-time synchronization of simulation data, including vehicle dynamics like location and speed, is achieved using the Traffic Control Interface (TraCI) in SUMO and the supervisor in Webots. The platform also incorporates physical input devices such as a Logitech G29 racing wheel and pedals, as well as an eye-tracker to capture the driver's eye movement. 

<p align="center">
  <b>Figure 1. Co-Simulation Platform Overview</b>
  <br>
  <img src="https://github.com/zhangcong0315/SUMO-and-Webots-co-simulation-platform/blob/main/Picture1.png" alt="Co-Simulation Platform Overview">
</p>

Figure 2 shows a snapshot of the co-simulation platform. The Webots view represents the simulation environment seen by the driver including the ego vehicle (controlled by the human driver through the Logitech Racing Wheel in Figure 2(H)), roadways, lane markings, and traffic signs (Figure 2(E)). Vehicle speed (Figure 2(D)) and warning information (when available) (Figure 2(C)) are also displayed in the ego vehicle. Additionally, drivers can utilize the rear mirror to assess the state of the vehicles behind them. In the SUMO view, the green vehicle is the representation of the ego vehicle in Webots while red vehicles are background traffic. Vehicle states in SUMO and Webots are synchronized.  A GP3 HD eye tracker manufactured by GazePoint is connected to the system (Figure 2(I) to capture the eye movements.![image](https://github.com/zhangcong0315/SUMO-and-Webots-co-simulation-platform/assets/61287266/9ba3c461-acff-4d41-ab4c-c895bd08306c)


<p align="center">
  <b>Figure 2. Multi-level driving simulator</b>
  <br>
  <img src="https://github.com/zhangcong0315/SUMO-and-Webots-co-simulation-platform/blob/main/Picture2.png" alt="Co-Simulation Platform Overview">
</p>


One notable application of this platform is the recreation of a real-world roundabout in Ann Arbor, Michigan, serving as the study area for investigating merging scenarios. The co-simulation platform enables the flexible generation of road networks based on real-world maps, manipulation of background vehicles in SUMO to simulate various driving scenarios, and interaction with the Webots vehicle, allowing for a dynamic and immersive driving experience.  Furthermore, the co-simulation platform allows the incorporation of designed countermeasures (e.g., warnings) into the Webots environment, which are then reflected in the driving simulator. In essence, this multi-level simulation platform integrates high-fidelity vehicle dynamics and inputs at the vehicle level, microscopic dynamics at the traffic level, and a detailed examination of human factors in different driving scenarios through the measurement of eye movements and maneuvers.Please see our paper for more detailed information and please cite it if you think this platform is helpfule https://arxiv.org/pdf/2312.03891v1.pdf.



