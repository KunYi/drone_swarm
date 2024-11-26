# Drone　Swarm

A sophisticated multi-drone formation control and visualization system using Ultra-Wideband (UWB) positioning technology. This project enables complex 3D formations and intelligent swarm behavior for drone fleets.

## Demo Video

[![Drone Swarm Formation Demo](https://img.youtube.com/vi/Gsv-Lswyug4/0.jpg)](https://youtu.be/Gsv-Lswyug4)

Click the image above to watch the demonstration video of the drone swarm formations in action.

## Features

- **Advanced Formation Control**
  - Multiple 3D formations (Ground Grid, Cube, Sphere, Pyramid, DNA Helix)
  - Smooth formation transitions
  - Dynamic target assignment
  - Real-time position updates

- **UWB Positioning System**
  - Precise position estimation
  - RANSAC-based multilateration
  - Error modeling and compensation
  - Time synchronization management

- **Safety Mechanisms**
  - Collision avoidance
  - Boundary protection
  - Velocity and acceleration limits
  - Multi-layer safety checks

- **Real-time Visualization**
  - 3D formation display
  - Motion trajectory tracking
  - Status monitoring
  - Interactive viewing angles

## Requirements

```
numpy>=1.21.0
matplotlib>=3.4.0
scipy>=1.7.0
scikit-learn>=1.5.0
```

## Installation

1. Clone the repository:
```bash
git clone https://github.com/KunYi/uwb-swarm.git
cd uwb-swarm
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

Run the main simulation:
```bash
python drone_formation.py
```

The simulation will demonstrate various formations in sequence:
1. Ground Grid Formation (5s)
2. Cubic Formation (10s)
3. Spherical Formation (10s)
4. Pyramid Formation (10s)
5. DNA Helix Formation (10s)

## System Parameters

- World Size: 40m x 40m x 40m
- Default Drone Count: 125
- Maximum Speed: 0.5 m/s
- Safe Distance: 1.5m
- Formation Duration: 2-10s per phase

## Contributing

Contributions are welcome! Please feel free to submit pull requests.

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

KUNYI CHEN (kunyi.chen@gmail.com)

## Acknowledgments

- Thanks to all contributors and users of this project
- Special thanks to the drone and UWB positioning research community
