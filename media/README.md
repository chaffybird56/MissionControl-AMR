# Media Directory

This directory contains media files for the ROS2 AMR project, including demo videos, screenshots, and documentation assets.

## Structure

```
media/
├── hero.gif          # Main demo video (30-60 seconds)
├── screenshots/      # Screenshots of the dashboard and system
│   ├── dashboard.png
│   ├── mission_panel.png
│   ├── map_view.png
│   └── metrics.png
└── README.md         # This file
```

## Demo Recording

To record a new demo:

1. Start the ROS2 AMR system:
   ```bash
   cd ops && make up
   ```

2. Record the demo:
   ```bash
   cd scripts && ./record_demo.sh
   ```

3. The recorded video will be saved to `media/ros2_amr_demo_TIMESTAMP.mp4`

## Screenshots

Screenshots should be taken of:
- Main dashboard interface
- Mission panel with active missions
- Map view showing robot navigation
- Metrics panel with performance data
- Webots simulation environment

## File Guidelines

- **Hero GIF**: Should be 30-60 seconds, showing the complete workflow
- **Screenshots**: PNG format, high resolution (1920x1080 or higher)
- **File naming**: Use descriptive names with underscores (e.g., `dashboard_overview.png`)

## Usage

These media files are used in:
- README.md documentation
- GitHub repository
- Presentation materials
- Project demonstrations

