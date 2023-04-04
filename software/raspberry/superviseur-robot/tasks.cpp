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
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 16
#define PRIORITY_TBATTERY 15
#define PRIORITY_TARENA 20
#define PRIORITY_TPOSITION 20
#define PRIORITY_TWATCHDOG 20
#define PRIORITY_TRELOADWATCHDOG 27


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
    if (err = rt_sem_create(&sem_startRobotWithoutWatchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobotWithWatchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reloadWatchDog, NULL, 0, S_FIFO)) {
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
    if (err = rt_task_create(&th_startRobotWithoutWatchdog, "th_startRobotWithoutWatchdog", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobotWithWatchdog, "th_startRobotWithWatchdog", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWatchDog, "th_reloaWatchDog", 0, PRIORITY_TRELOADWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
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
    if (err = rt_task_start(&th_startRobotWithWatchdog, (void(*)(void*)) & Tasks::StartRobotTaskWithWatchdog, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobotWithoutWatchdog, (void(*)(void*)) & Tasks::StartRobotTaskWithoutWatchdog, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWatchDog,(void(*)(void*)) & Tasks::reloadWatchDog, this)){
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
    camera.Close();
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
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
 
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
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
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
        delete(msgRcv); // mus be deleted manually, no consumer
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
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
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
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

void Tasks::GetVBat(void * arg) {
    
    int rs;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    rt_task_set_periodic(NULL,TM_NOW,500000000);
    while(1){
        rt_task_wait_period(NULL);
        cout << "Periodic battery level" << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            
            //Ask the battery level to the robot:
            MessageBattery * levelBat;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            levelBat = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            // Send the message:
            WriteInQueue(&q_messageToMon, levelBat);
        }
        cout << endl << flush;
    }
}

void Tasks::ManageCamera(void * arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    bool isOpen;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1){
        // Fonctionnalité 14
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
        // Fonctionnalité 15
        rt_task_set_periodic(NULL,TM_NOW,100000000); // envoi toutes les 100 ms
        while(1){
            if (!camera.IsOpen()) break; // if camera's closed, stop img acquisition
            rt_task_wait_period(NULL);
            cout << "CAMERA : Periodic image acquisition" << endl << flush;
            rt_mutex_acquire(&mutex_camera,TM_INFINITE);
            Img * img = new Img(camera.Grab());
            if(!arena.IsEmpty()) {
                cout << "CAMERA : Periodic image acquisition with camera" << endl << flush;
                rt_mutex_acquire(&mutex_arena,TM_INFINITE);
                img ->DrawArena(arena);
                rt_mutex_release(&mutex_arena);                       
            }
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
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgImg);
            rt_mutex_release(&mutex_monitor);
            rt_mutex_release(&mutex_camera);
            cout << endl << flush;
        }
    }
}

void Tasks::StopCamera(void * arg){
    bool isOpen;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1){
        // Fonctionnalité 16
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
 
void Tasks::SetupArena(void * arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    //Fonctionnalité 17
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
            } else if (rcvArena == MESSAGE_CAM_ARENA_INFIRM){
            }
        }
        rt_mutex_release(&mutex_camera);
    }
}

void Tasks::RobotPosition(void * arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    // Fonctionnalité 18
    while (1) {
        rt_sem_p(&sem_robotPosition,TM_INFINITE);
        rt_mutex_acquire(&mutex_imageType,TM_INFINITE);
        imgType = 1;
        rt_mutex_release(&mutex_imageType);
        cout << "SUCCESS : Position enabled" << endl << flush;
    }
}

void Tasks::StopPosition(void * arg) {
    cout << "Start" << __PRETTY_FUNCTION__ << endl << flush;
    //Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    // Fonctionnalité 19
    while(1) {
        rt_sem_p(&sem_stopPosition, TM_INFINITE);
        rt_mutex_acquire(&mutex_imageType,TM_INFINITE);
        imgType = 0;
        rt_mutex_release(&mutex_imageType);
        cout << "SUCCESS : Position disabled" << endl << flush;
    }
}

void Tasks::StartRobotTaskWithoutWatchdog(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    int count = 0;
    Message * msgSend;

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
            rt_sem_p(&sem_startRobotWithoutWatchdog, TM_INFINITE);
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
        
        
        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            count = 0;
        }
        else{
            count += 1;
            if (count==3){
                Message * msg = new Message(MESSAGE_MONITOR_LOST);
                monitor.Write(msg);
                robot.Close();
                robot.Write(robot.Reset());
                count = 0;
            }
        }
    }
}

void Tasks::StartRobotTaskWithWatchdog(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    Message * msgSend;
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    
    rt_task_set_periodic(NULL, TM_NOW, 100000000000);
    while (1) { 
        rt_sem_p(&sem_startRobotWithWatchdog, TM_INFINITE);
        cout << "Start robot with watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
        
           /* each second reload the watchdog*/
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
        else{
            cout << "Robot failed to start" << endl << flush;    
        }
        rt_sem_v(&sem_reloadWatchDog);
        
        
   /*     while (1) {  
            rt_task_wait_period(NULL);
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            cout << "mutex aqcuired for watchdog realoding" << endl << flush;
            if(robotStarted == 1) {
                cout << "in if statement" << endl << flush;
                
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msg = robot.Write(robot.ReloadWD());
                rt_mutex_acquire(&mutex_robot,TM_INFINITE);
                if (msg ->GetID() == MESSAGE_ANSWER_ACK){
                    cout << "WD Reloaded succecefully" << endl << flush;
                }
                else{
                    cout << "WD failed to Reload" << endl << flush;
                }
            }
            else{break;} 
            rt_mutex_release(&mutex_robotStarted);
        }   */
                
    }
}              

void Tasks::reloadWatchDog(void *arg){
    Message *msg;
    int count = 0 ;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    int rs;
    
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    while (1) {  
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        //cout << "mutex aqcuired for watchdog realoding" << endl << flush;
        if(rs == 1) {
            //cout << "in if statement" << endl << flush;            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);
            if (msg ->GetID() == MESSAGE_ANSWER_ACK){
               // cout << "WD Reloaded succecefully" << endl << flush;
            }
            else{
                count += 1;
                if (count ==3){
                    Message * msg = new Message(MESSAGE_MONITOR_LOST);
                    monitor.Write(msg);
                    robot.Close();
                    robot.Write(robot.Reset());
                    count = 0;
                }
            }
                //cout << "WD failed to Reload" << endl << flush;
        }
    }
}  

