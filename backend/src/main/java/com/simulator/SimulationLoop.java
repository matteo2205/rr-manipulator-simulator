package com.simulator;

public class SimulationLoop implements Runnable{
    private boolean running = false;
    private final int targetTickRate = 50; //standard di aggiornamento del processo
    private final int frameTimeMillis = 1000/targetTickRate; //durata effettiva di ogni aggiornamento
    private RobotRR robot;

   public SimulationLoop(RobotRR robot){
    this.robot = robot; //definizione dell'oggetto che passa al loop
   }

   public synchronized void start(){
    if(running){ return; } //se esiste siamo già nel loop non viene riavviato
    running = true; //imposta a true per rimanere nel loop
    Thread simulationThread = new Thread(this, "Simulation Thread");
    simulationThread.start();
    System.out.println("[SimulationLoop] -> motore di simulazione avviato a " + targetTickRate + "Hz"); //messaggio di controllo
   }

   public synchronized void stop(){
    running = false; //per interrompere il loop
    System.out.println("[SimulationLoop] -> Arresto del motore di simulazione");
   }

   public void run(){
    long lastTime = System.currentTimeMillis(); 

    while(running){
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime)/1000.0;
        lastTime=currentTime;
        
        updateSimulation(deltaTime);

        long elapsed = System.currentTimeMillis() - currentTime;
        long sleepTime = frameTimeMillis - elapsed;

        if(sleepTime>0){
            try{
                Thread.sleep(sleepTime);
                } catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                    System.out.println("[SimulationLoop] -> thread interrotto inaspettatamente");
                    break;
                }
        }
    }
    }

    //metodo di controllo e gestione interna per evitare errori di sistema nel caso in cui il manipolatore non esista ancora effettivamente
    private void updateSimulation(double dt){ 
        if(robot != null){
            robot.update(dt);
        }
    }
}