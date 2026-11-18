package com.simulator;

public class Main {
    public static void main(String[] args) {
        System.out.println("==========================================");
        System.out.println("   AVVIO BACKEND SIMULATORE MANIPOLATORE 2R ");
        System.out.println("==========================================");

        // 1. Istanzia il modello cinematico e dinamico del robot
        RobotRR robot = new RobotRR();

        // 2. Configura e avvia il loop di simulazione in tempo reale a 50Hz
        SimulationLoop simulationLoop = new SimulationLoop(robot);
        Thread simThread = new Thread(simulationLoop, "Simulation-Thread");
        simThread.start();
        System.out.println("[MAIN] -> SimulationLoop avviato a 50Hz con successo.");

        // 3. Avvia il server WebSocket sulla porta 8887 (comunicazione col frontend)
        int port = 8887;
        SimulatorServer server = new SimulatorServer(port, robot);
        server.start();
        System.out.println("[MAIN] -> Server WebSocket in ascolto sulla porta: " + port);
        
        System.out.println("==========================================");
        System.out.println("   SISTEMA OPERATIVO E PRONTO ALL'USO     ");
        System.out.println("==========================================");
    }
}