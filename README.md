# ATOM (Autonomous Task Oriented Machine)

- Bachelors FYP (Final Year Project) done in the semesters Fall 2015 and Spring 2016 at FAST-NU, Lahore

## Abstract
In this project, we tried to turn a basic robotic platform into an Autonomous Task Oriented Machine (ATOM). The Autonomous robot that we developed is able to navigate itself through a specified area and perform tasks assigned to it, autonomously. Using any defined mechanism, a (transportation) task will be assigned to the robot and based on its internal priority-based system, the robot will perform all the tasks assigned to it and will make sure to avoid collisions of any kind by detecting and mapping the area around it.
To accomplish this objective, we did mapping, localization, path finding, obstacle avoidance and data transmission from a user’s device (laptop or mobile phone) to the robot. We used Raspberry pi, ultrasonic sensors, stepper motors, Wi-Fi module and a power bank and for the coding has been done in Python.

## 1. Introduction
Liberty Lab of FAST University owns a robot car that was automated by groups from both Electrical Engineering (EE) and Computer Science (CS) departments. Now the next step in this continuous process of making a better robotic car, was to make it autonomous. In our FYP, we aimed to achieve this next step and hope to make this robotic car completely autonomous and able to perform tasks assigned to it.
Not only would making this autonomous robotic car be a big step academically but also this robot would help to perform the daily tedious tasks of taking things from a specified location to another, that normally requires the services of a human being. The robot would detect obstacles in its path and would move around the obstacles to reach the destination and would avoid collision at all costs.
We hope to provide a user-friendly interface for the faculty members and other people that are going to initially use this robot for performing transportation tasks, We will also implement a priority system, based on real life, so that for example, a higher priority faculty member gets his task done in relatively less time than a faculty member with a lower priority.

## 2. Requirements and Design  

### 2.1 User Characteristics
There will only be one kind of users of our project and thus the concept of user’s role is ruled out. The description of one such user is given below:
   - The user of our project would be a staff member or administrator in FAST National University. The user will assign a location, from the available choices, for performing a transport task to the robot through an application interface and ATOM would then, according to its internal working and priority, perform the task.

### 2.2 Domain Overview
We are aiming to make a robotic toy car, owned by Liberty Lab of FAST-NU, completely autonomous and able to perform transportation tasks assigned to it, by itself without any external help. The robot would detect obstacles in its path and would move around the obstacles to reach the destination and would avoid collision at all costs. We hope to provide a user-friendly interface for the faculty members and other people that are going to initially use this robot for performing transportation tasks. We will also implement a priority system, based on real life, so that for example, a higher ranking faculty member gets his task done in relatively less time than a faculty member with a lower rating.

### 2.3 Functional Requirements

1. System shall allow the user to Sign up using university email id. New user would need confirmation of his/her account from system managers.
2. System shall maintain a task-priority level for every user, based on real life.
3. System shall allow the user to ask ATOM to perform a transport task.
   - User will provide the source and destination location for the task.
   - User will also be able to set an optional urgency level for the given task.
4. System shall allow the user to Log in and Log out.

### 2.4 Hardware/Software Requirements

Following are the software and hardware requirements that are necessary to develop and deploy ATOM: 

#### Hardware Requirements

- Raspberry Pi.
- ARM Discovery.
- Bluetooth Module for Discovery.
- Ultrasonic Sensors.
- Toy Car.
- 24V battery for powering ATOM.

#### Software Requirements

- Raspbian OS for Pi.
- Android OS for Client Application.

## 3. Implementation

![System Design](https://github.com/hmhamza/autonomous-task-oriented-machine-bs-final-year-project/blob/master/System%20Design.jpg)

### Figure: System Design

User command is sent to the pi from the android phone via an internet router. The pi then sends the respective commands to the tires/motors. Meanwhile, the ultrasonic sensors keep on taking readings for localization and obstacle avoidance.

### 3.1 Mapping

The first work that we did was mapping. The robot traversed the whole path where it is to be deployed and recorded the ultrasonic readings in a file. These reading were basically a measure of where are the walls in the path. These records are kept in the file.


Then this generated map was converted to bitmap because this map takes a lot of space which we can’t afford in the pi. Plus, the processing time can be slowed down because it would take a lot of time in the comparison in localization.

### 3.2 Localization

Our next task to use this map for the localization of robot. The problem was that if we place the robot anywhere in the path, the robot should know where it’s currently located.


This will be used when someone wants to call the car to himself or wants the car to carry his things from that source to the destination.


```
1. Take readings from the sensor at the current location which is to be localized
2. Move the robot 1m forward and again take the reading
3. Repeat Step 2
4. Compare these 3 readings with every group of 3 readings in the MAP file/database
5. The maximum match will be the location of the robot
```
#### Pseudo code of Localization

### 3.3 Path Finding

Then our next step was to find the path from the source to the destination using the map file we developed in the first step. Firstly we used DFS for this purpose but considering the point that the robot can be deployed in any type of area, we replaced our algorithm with the Dijkstra’s algorithm to find the shortest path from the source to the destination.

```
1. Initialize DijkstraTable[] for Dijkstra Algorithm
2. Add all nodes of the graph to nodesQueue[]
3. while nodesQueue is not empty
4.      Extract cheapest element from the nodesQueue as u
5.      Relax edges of u
6. Develop the final path using the DijkstraTable[] i.e. put the respective direction, distance and the checkpoint
```
#### Pseudo code of Mapping

### 3.4 Obstacle Avoidance

On its path from source to the destination, the robot may meet several obstacles. These obstacles may affect its performance and might also damage the car.


To solve this problem, we installed some extra S sensors on the robot which took continuous readings from the surroundings and the pi then processes those reading to decide whether or not there is an obstacle nearby.
```
threads = []
	
# Create new threads
thread1 = myThread(1, "Left Sensor Thread", LeftSensorPins)
thread2 = myThread(1, "Right Sensor Thread", RightSensorPins)
	
# Start new Threads
thread1.start()
thread2.start()

# Add threads to thread list
threads.append(thread1)
threads.append(thread2)
```
#### Creation and execution of Obstacle Avoidance threads

### 3.5 Sending data to the pi

The final step was to develop a platform for a user to communicate with the pi i.e. to tell the robot what to do. For this purpose, we developed an Android app that sends the data to a Wi-Fi router. The Wi-Fi router then send that data to the Wi-Fi module that we installed on pi to receive data from Wi-Fi routers. Then, that data is read and processed, and then it’s decided which task has to be performed.


## 4. Conclusions

To summarize it all, we have accomplished all of the main tasks that we set out to accomplish about 9 months ago. We performed different types of hardware work that we had never done before and that was one of the things that we got to learn during our Final Year Project. As interesting as this new hardware work was for us, it wasn’t without its difficulties and complications, which I’ll try to explain in the paragraph below.
As mentioned before in our reports, our project was continuation of EE project which was to automate the car. But unfortunately, when we got the car, it wasn’t working as it was supposed to work. So the first stage in our project became to automate this car. Now we tried our best, along with the help of Sir Yasir Niaz Khan, but the hardware problems were too many to solve. So after trying for almost 4 months, we had to move to a smaller robotic car and from then on, we did our whole project on that car.
Now the most interesting part of our FYP is that it can be extended to perform a lot of tasks, since the basic autonomous structure with obstacle avoidance and task assignment has been firmly laid. Some of the many extensions that can happen are recommended as given below:
- Facial recognition using camera for personal task assignments.
- Using the above mentioned facial recognition for security purposes.
- Using ATOM as the tour guide for newcomers to FAST University.
- Implementing Advanced AI techniques to explore unmapped areas by itself.

