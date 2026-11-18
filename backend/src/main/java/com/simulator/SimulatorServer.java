package com.simulator;

import org.java_websocket.server.WebSocketServer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import java.net.InetSocketAddress;

public class SimulatorServer extends WebSocketServer {
    private RobotRR robot; 

    public SimulatorServer(int port, RobotRR robot) {
        super(new InetSocketAddress(port));
        this.robot = robot;
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) { 
        System.out.println("[Server] -> Nuovo client connesso da: " + conn.getRemoteSocketAddress()); 
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) { 
        System.out.println("[Server] -> Client disconnesso: " + conn.getRemoteSocketAddress()); 
    }

    @Override
    public void onMessage(WebSocket conn, String message) { 
        System.out.println("[Server] -> Messaggio ricevuto dal frontend: " + message);
        
        if (message != null && !message.isEmpty()) {
            System.out.println("[Server] -> Stato attuale end-effector X: " + robot.getEndEffectorX());
        }
    }

    @Override
    public void onError(WebSocket conn, Exception ex) { 
        System.err.println("[Server] -> Errore riscontrato nel socket:"); 
        ex.printStackTrace();
    }

    @Override
    public void onStart() { 
        System.out.println("[Server] -> Server WebSocket avviato con successo e pronto sulla porta: " + getPort()); 
    }
}