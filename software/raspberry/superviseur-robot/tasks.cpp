/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 24
#define PRIORITY_TSENDTOMON 25
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 16
#define PRIORITY_TBATTERY 15
#define PRIORITY_TARENA 20
#define PRIORITY_TPOSITION 20
#define PRIORITY_TWATCHDOG 20
#define PRIORITY_TERROR 20


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_imageType, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_watchdog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_problemComRobot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_setupArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_confirmArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_robotPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_comRobotError, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_watchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task */create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startCamera, "th_startCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stopCamera, "th_stopCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_setupArena, "th_setupArena", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_robotPosition, "th_robotPosition", 0, PRIORITY_TPOSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stopPosition, "th_stopPosition", 0, PRIORITY_TPOSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_watchdog, "th_watchdog", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /* if (err = rt_task_create(&th_watchError, "th_watchError", 0, PRIORITY_TERROR, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } */
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;
    

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetVBat, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startCamera, (void(*)(void*)) & Tasks::ManageCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stopCamera, (void(*)(void*)) & Tasks::StopCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_setupArena, (void(*)(void*)) & Tasks::SetupArena, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_robotPosition, (void(*)(void*)) & Tasks::RobotPosition, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stopPosition, (void(*)(void*)) & Tasks::StopPosition, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_watchdog,(void(*)(void*)) & Tasks::WatchDog, this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE); 
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
    if (camera.IsOpen()) {camera.Close();}
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    int errorCounter = 0;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {
        cout << "Wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
        /**
        * Feature 8 & 9 -- watching communication errors with robot
        *
        * This function counts the number of consecutive errors during communication with robot.
        * It closes communication if this number is above 3, putting it in an initial state.
        *
        */
        if ((msg->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND)) ||
            (msg->CompareID(MESSAGE_ANSWER_ROBOT_ERROR)) ||
            (msg->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT))) {
            errorCounter ++;
            if (errorCounter >= 3) {
                cout << "Connection with robot compromised" << endl << flush;
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Write(robot.Stop());
                robot.Write(robot.Reset());
                robot.Close();
                rt_mutex_release(&mutex_robot);
                errorCounter = 0;
            } else errorCounter = 0;
        }
    }
   
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            // Feature 5 && 6 -- catching communication error message with monitor, stop & close robot & camera
            cout << "Lost communication with monitor" << endl << flush;
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(robot.Stop());
            robot.Write(robot.Reset());
            robot.Close();
            rt_mutex_release(&mutex_robot);
            rt_sem_v(&sem_stopCamera);
            delete(msgRcv);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            watchdog = 0;
            rt_mutex_release(&mutex_watchdog);
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            watchdog = 1;
            rt_mutex_release(&mutex_watchdog);
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_sem_v(&sem_openCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_sem_v(&sem_stopCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_setupArena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM) || 
                msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            arena_confirm = msgRcv ->GetID();
            rt_mutex_release(&mutex_arena);
            rt_sem_v(&sem_confirmArena);
        }
        else if (msgRcv ->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_sem_v(&sem_robotPosition);
        }
        else if (msgRcv ->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_sem_v(&sem_stopPosition);
        }
        delete(msgRcv); // must be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;
        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "hello" << endl << flush;
        int wd = 0;
        rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
        wd = watchdog;
        rt_mutex_release(&mutex_watchdog);
        
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        if (wd) {
            cout << "Start robot with watchdog (" << flush;
            msgSend = robot.Write(robot.StartWithWD());
            cout << msgSend->GetID() << flush;
            cout << ")" << endl << flush;
            rt_sem_v(&sem_watchdog);
        }
        else {
            cout << "Start robot without watchdog (";
            msgSend = robot.Write(robot.StartWithoutWD());
            cout << msgSend->GetID();
            cout << ")" << endl;
        }
        rt_mutex_release(&mutex_robot);
        

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
        
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
        cout << "hello2" << endl << flush;
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }
    return msg;
}

/**
 *  Feature 13 -- periodic battery level acquisition
 * 
 * This function periodically acquires robot's battery level when asked by the monitor
 */
void Tasks::GetVBat(void * arg) {
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL,TM_NOW,500000000);
    while(1){
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            //Ask the battery level to the robot:
            MessageBattery * levelBat;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            levelBat = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(levelBat);
            rt_mutex_release(&mutex_monitor);
        }
        else {
            cout << "Robot not started" << endl << flush; 
        }
        cout << endl << flush;
    }
}

/**
* Feature 14 -- open camera
* Feature 15 -- periodic image acquisition
*
* This function opens the camera and starts periodic image acquisition (10 fps)
* Image can have an arena drawn (view SetupArena task)
* Image can have the position of a robot marked (view RobotPosition task)
*/
void Tasks::ManageCamera(void * arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    bool isOpen;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1){
        // Feature 14
        rt_sem_p(&sem_openCamera,TM_INFINITE);
        cout << "Trying to open Camera" << endl << flush;
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        isOpen=camera.Open();
        rt_mutex_release(&mutex_camera);
        if (!isOpen){
            cout << "ERROR : Camera could not open" << endl << flush;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new Message(MESSAGE_ANSWER_NACK));
            rt_mutex_release(&mutex_monitor);
            throw std::runtime_error{"Error camera could not open"};
        } else {
            cout << "SUCCESS : Opening Camera" << endl << flush;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new Message(MESSAGE_ANSWER_ACK));
            rt_mutex_release(&mutex_monitor);
        }
        // Feature 15
        rt_task_set_periodic(NULL,TM_NOW,100000000); // periodic : every 100 ms
        while(1){
            if (!camera.IsOpen()) break; // if camera is closed, stop img acquisition
            rt_task_wait_period(NULL);
            cout << "CAMERA : Periodic image acquisition" << endl << flush;
            rt_mutex_acquire(&mutex_camera,TM_INFINITE);
            Img * img = new Img(camera.Grab());
            if(!arena.IsEmpty()) {
                cout << "CAMERA : Periodic image acquisition with camera" << endl << flush;
                rt_mutex_acquire(&mutex_arena,TM_INFINITE);
                img ->DrawArena(arena);
                rt_mutex_release(&mutex_arena);                       
                if(imgType==1){
                    cout << "CAMERA : Periodic position acquisition" << endl << flush;
                    std::list<Position> robotPos;
                    robotPos = img->SearchRobot(arena);
                    if (!robotPos.empty()) {
                        Position robot1 = robotPos.front();
                        img->DrawRobot(robot1);
                        MessagePosition *msgPos = new MessagePosition(MESSAGE_CAM_POSITION, robot1);
                        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                        monitor.Write(msgPos);
                        rt_mutex_release(&mutex_monitor);
                    }
                }
            }
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgImg);
            rt_mutex_release(&mutex_monitor);
            rt_mutex_release(&mutex_camera);
            cout << endl << flush;
        }
    }
}

/** 
* Feature 16 -- stop image acquisition
*
* This function stops image acquisition when asked by the monitor
*/
void Tasks::StopCamera(void * arg){
    bool isOpen;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1){
        rt_sem_p(&sem_stopCamera,TM_INFINITE);
        cout << "Trying to close Camera" << endl << flush;
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        camera.Close();
        rt_mutex_release(&mutex_camera);
        isOpen= camera.IsOpen();
        if (isOpen){
            cout << "ERROR : Camera could not close" << endl << flush;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new Message(MESSAGE_ANSWER_NACK));
            rt_mutex_release(&mutex_monitor);
            throw std::runtime_error{"Error camera could not close"};
        } else {
            cout << "SUCCESS : Camera closed" << endl << flush;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new Message(MESSAGE_ANSWER_ACK));
            rt_mutex_release(&mutex_monitor);
        }
    }
}
 
/**
* Feature 17 -- setup Arena outline
* 
* This Function searches an arena on last acquired image. If found, it is drawn and kept on future images, else nothing is saved nor drawn
*
*
*/ 
void Tasks::SetupArena(void * arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_setupArena,TM_INFINITE);
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        Img * img = new Img(camera.Grab());
        Arena foundArena = img->SearchArena();
        if (foundArena.IsEmpty()){
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new Message(MESSAGE_ANSWER_NACK));
            rt_mutex_release(&mutex_monitor);
        } else {
            img->DrawArena(foundArena);
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgImg);            
            rt_mutex_release(&mutex_monitor);
            rt_sem_p(&sem_confirmArena,TM_INFINITE);
            rt_mutex_acquire(&mutex_arena,TM_INFINITE);
            int rcvArena = arena_confirm;
            rt_mutex_release(&mutex_arena);
            if (rcvArena == MESSAGE_CAM_ARENA_CONFIRM) {
                cout << "SUCCESS : Confirming Arena" << endl << flush;
                arena = foundArena;
            } else if (rcvArena == MESSAGE_CAM_ARENA_INFIRM){/*theoretically, we should delete saved arena*/}
        }
        rt_mutex_release(&mutex_camera);
    }
}

/**
* Feature 18 -- get robot's position 
*
* This function enables position acquisiton
*/
void Tasks::RobotPosition(void * arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_robotPosition,TM_INFINITE);
        rt_mutex_acquire(&mutex_imageType,TM_INFINITE);
        imgType = 1;
        rt_mutex_release(&mutex_imageType);
        cout << "SUCCESS : Position enabled" << endl << flush;
    }
}

/** 
* Feature 19 -- stop getting robot's position
*
* This function disables position acquisition
*/
void Tasks::StopPosition(void * arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    //Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while(1) {
        rt_sem_p(&sem_stopPosition, TM_INFINITE);
        rt_mutex_acquire(&mutex_imageType,TM_INFINITE);
        imgType = 0;
        rt_mutex_release(&mutex_imageType);
        cout << "SUCCESS : Position disabled" << endl << flush;
    }
}

/** 
* Feature 11 -- Watchdog
*
* This function allows the robot to start with Watchdog disabled / enabled depending on 
*/
void Tasks::WatchDog (void * arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    //Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while(1) {
        rt_sem_p(&sem_watchdog,TM_INFINITE);
        rt_task_set_periodic(NULL,TM_NOW,1000000000);
        while (1) {
            rt_task_wait_period(NULL);
            Message * msgSend;
            int rs = 0;
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            if (rs==1) {
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(robot.ReloadWD());
                rt_mutex_release(&mutex_robot);
                cout << "WD : " << msgSend->ToString() << endl << flush;
                delete(msgSend);
            }            
        }
        
    }
}