# robotics-26-27

## Set up project

> [!CAUTION]
> This workflow is intended for MacOS. It will not work on Windows or Linux.

There are two possible configurations for this project: Intermediate and Advanced. This repository uses both. Start with "Intermediate" then move on to "Advanced".

**Dependencies:** git, uv, docker.

### Intermediate 

Clone the repository, sync the uv dependencies and do a pytest run.
```bash
git clone <url>
cd robotica-grupo01
uv sync
uv run pytest test
```

### Advanced

Set "ROS_DOMAIN_ID" and "ROBOT_IP" in the .env file.
Then, build the image:
```bash
cp .env.example .env
./lab.sh build
```

Start docker and check for errors:
```bash
./lab.sh up
./lab.sh doctor
```

Build ROS packages:
```bash
./lab.sh shell          
lab build
exit            
```

Run the firmware simulator:
```bash
./firmware_sim.sh
```

### Running a script

```bash
uv run python -m scripts.<script_name>
```