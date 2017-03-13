/**
 * This program runs as a server and controls the force to be applied to balance the Inverted Pendulum system running on the clients.
 */
import java.io.*;
import java.net.*;
import java.util.*;

public class ControlServer {

    private static ServerSocket serverSocket;
    private static final int port = 25533;

    /**
     * Main method that creates new socket and PoleServer instance and runs it.
     */
    public static void main(String[] args) throws IOException {
        try {
            serverSocket = new ServerSocket(port);
        } catch (IOException ioe) {
            System.out.println("unable to set up port");
            System.exit(1);
        }
        System.out.println("Waiting for connection");
        do {
            Socket client = serverSocket.accept();
            System.out.println("\nnew client accepted.\n");
            PoleServer_handler handler = new PoleServer_handler(client);
        } while (true);
    }
}

/**
 * This class sends control messages to balance the pendulum on client side.
 */
class PoleServer_handler implements Runnable {
    // Set the number of poles
    private static final int NUM_POLES = 3;

    static ServerSocket providerSocket;
    Socket connection = null;
    ObjectOutputStream out;
    ObjectInputStream in;
    String message = "abc";
    static Socket clientSocket;
    Thread t;



    /**
     * Class Constructor
     */
    public PoleServer_handler(Socket socket) {
        t = new Thread(this);
        clientSocket = socket;
        try {
            out = new ObjectOutputStream(clientSocket.getOutputStream());
            out.flush();
            in = new ObjectInputStream(clientSocket.getInputStream());
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        t.start();
    }
    double angle, angleDot, pos, posDot, action = 0, i = 0;

    /**
     * This method receives the pole positions and calculates the updated value
     * and sends them across to the client.
     * It also sends the amount of force to be applied to balance the pendulum.
     * @throws ioException
     */
    void control_pendulum(ObjectOutputStream out, ObjectInputStream in) {
        try {
            while(true){
                System.out.println("-----------------");

                // read data from client
                Object obj = in.readObject();

                // Do not process string data unless it is "bye", in which case,
                // we close the server
                if(obj instanceof String){
                    System.out.println("STRING RECEIVED: "+(String) obj);
                    if(obj.equals("bye")){
                        break;
                    }
                    continue;
                }
                
                double[] data= (double[])(obj);
                assert(data.length == NUM_POLES * 4);
                double[] actions = new double[NUM_POLES];
 
                // Get sensor data of each pole and calculate the action to be
                // applied to each inverted pendulum
                // TODO: Current implementation assumes that each pole is
                // controlled independently. This part needs to be changed if
                // the control of one pendulum needs sensing data from other
                // pendulums.
                for (int i = 0; i < NUM_POLES; i++) {
                  angle = data[i*4+0];
                  angleDot = data[i*4+1];
                  pos = data[i*4+2];
                  posDot = data[i*4+3];
//                    angle = data[0*4+0];
//                    angleDot = data[0*4+1];
//                    pos = data[0*4+2];
//                    posDot = data[0*4+3];
                    
                  System.out.println("server < pole["+i+"]: "+angle+"  "
                      +angleDot+"  "+pos+"  "+posDot);
                  if(i == 0)
                	  actions[i] = calculate_action(angle, angleDot, pos, posDot);
                  else if(i == 1)//if the second pendulum, then also send the pos of the first
                	  actions[i] = calculate_action2(angle, angleDot, pos, posDot, data[0*4+2]);
                  else  // if the thrid pendelum, then also send the pos of the second
                	  actions[i] = calculate_action3(angle, angleDot, pos, posDot, data[1*4+2]);
                  }
                sendMessage_doubleArray(actions);

            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }

        try {
            if (clientSocket != null) {
                System.out.println("closing down connection ...");                
                out.writeObject("bye");
                out.flush();
                in.close();
                out.close();
                clientSocket.close();
            }
        } catch (IOException ioe) {
            System.out.println("unable to disconnect");
        }

        System.out.println("Session closed. Waiting for new connection...");

    }

    /**
     * This method calls the controller method to balance the pendulum.
     * @throws ioException
     */
    public void run() {

        try {
            control_pendulum(out, in);

        } catch (Exception ioException) {
            ioException.printStackTrace();
        } finally {
        }

    }

    // Calculate the actions to be applied to the inverted pendulum from the
    // sensing data.
    // TODO: Current implementation assumes that each pole is controlled
    // independently. The interface needs to be changed if the control of one
    // pendulum needs sensing data from other pendulums.
    double calculate_action(double angle, double angleDot, double pos, double posDot) {
    	double kp = 11;
    	double kd = 1.2;   	
    	/* to move the pendelum forward, we add a bias to the
        angle which in this case serves as the error for the
        controller. This bias is determined by a linear function
        where the largest the bias will be is -.1 (this occurs when
        the pendelum is farthest from the desired location) and the
        smallest is 0 (this occurs when the pendelum is right on top
        of the desired location, 2 in this case)*/
    	double bias = (0.1/4)*pos - 0.05; 
        double angError = angle + bias;
        double posError = posDot;
        double action = 0;
        /* Our implementation of the PD controller involved using all four 
         * variables provided. This specific controller will move the penduelum
         * to the desired location, 2 in this case.
         */
        action = kp*angError + kd*angleDot + 1*posError + 0*pos;
        return action;
   }
    /*Function called for the second pendelum */
    double calculate_action2(double angle, double angleDot, double pos, double posDot , double pos2) {
    	/* This second controller made the second pendelum will follow the first
    	 * pendelum, staying exactly 0.6 behind the first pendelum, which is found
    	 * with the parameter pos2 (position of first pendelum). Since in this case,
    	 * the desired location is always moving, we have to calculate slope, and 
    	 * do a linear regression to find the corect bias for the angle
    	 */
    	double kp = 11;
    	double kd = 1.2;
    	/*
    	 * In the case for the second pendulum, the desired location is 0.6 away from the
    	 * first pendelum
    	 */
    	double slope = (-0.1)/(-3 - (pos2 - 0.6));
    	double bias = slope*(pos + 3) - 0.1 ;
        double angError = angle + bias;
        double posError = posDot;
        double action = 0;
        action = kp*angError + kd*angleDot + 1*posError + 0*pos;
        return action;
   }
    /*Function called for third pendelum, which it 0.6 away from the second pendelum */
    double calculate_action3(double angle, double angleDot, double pos, double posDot , double pos3) {
    	double kp = 11;
    	double kd = 1.2;
    	double slope = (-0.1)/(-4 - (pos3 - 0.6)); 
    	double bias = slope*(pos + 4) - 0.1 ;
        double angError = angle + bias;
        double posError = posDot;
        double action = 0;
        action = kp*angError + kd*angleDot + 1*posError + 0*pos;
        return action;
   }

    /**
     * This method sends the Double message on the object output stream.
     * @throws ioException
     */
    void sendMessage_double(double msg) {
        try {
            out.writeDouble(msg);
            out.flush();
            System.out.println("server>" + msg);
        } catch (IOException ioException) {
            ioException.printStackTrace();
        }
    }

    /**
     * This method sends the Double message on the object output stream.
     */
    void sendMessage_doubleArray(double[] data) {
        try {
            out.writeObject(data);
            out.flush();
            
            System.out.print("server> ");
            for(int i=0; i< data.length; i++){
                System.out.print(data[i] + "  ");
            }
            System.out.println();

        } catch (IOException ioException) {
            ioException.printStackTrace();
        }
    }


}
