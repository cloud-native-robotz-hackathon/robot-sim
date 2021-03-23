// File:          MyController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import org.rapidoid.buffer.Buf;
import org.rapidoid.bytes.BytesUtil;
import org.rapidoid.http.AbstractHttpServer;
import org.rapidoid.http.HttpStatus;
import org.rapidoid.http.MediaType;
import org.rapidoid.net.Server;
import org.rapidoid.net.abstracts.Channel;
import org.rapidoid.net.impl.RapidoidHelper;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class JavaController extends AbstractHttpServer {

  private static final byte FORWARD[] = "/forward".getBytes();
  private static final byte LEFT[] = "/left".getBytes();
  private static final byte RIGHT[] = "/right".getBytes();
  private static final byte BACK[] = "/back".getBytes();
  private static final byte RESPONSE[] = "OK".getBytes();

  static Motor motor1;
  static Motor motor2;

  @Override
  protected HttpStatus handle(Channel ctx, Buf buf, RapidoidHelper req) {

    if (req.isGet.value) {
      
      System.out.println(this.hashCode() + " : Incoming Request -> " + req.path);
      if (BytesUtil.startsWith(buf.bytes(), req.path, FORWARD, false)) {
                
        System.out.println(this.hashCode() + " : Action -> forward");
        motor1.setPosition(160.0);
        motor2.setPosition(160.0);

        return ok(ctx, req.isKeepAlive.value, RESPONSE, MediaType.TEXT_PLAIN);
      } else if (matches(buf, req.path, LEFT)) {

        System.out.println(this.hashCode() + " : Action -> left");
        motor1.setPosition(160.0);
        motor2.setPosition(160.0);

        return ok(ctx, req.isKeepAlive.value, RESPONSE, MediaType.TEXT_PLAIN);
      } else if (matches(buf, req.path, RIGHT)) {

        System.out.println(this.hashCode() + " : Action -> right");
        motor1.setPosition(160.0);
        motor2.setPosition(160.0);

        return ok(ctx, req.isKeepAlive.value, RESPONSE, MediaType.TEXT_PLAIN);
      } else if (matches(buf, req.path, BACK)) {

        System.out.println(this.hashCode() + " : Action -> back");
        motor1.setPosition(160.0);
        motor2.setPosition(160.0);

        return ok(ctx, req.isKeepAlive.value, RESPONSE, MediaType.TEXT_PLAIN);
      }
    }

    return HttpStatus.NOT_FOUND;
  }




  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {

    // create the Robot instance.
    Robot robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    motor1 = robot.getMotor("left wheel");
    motor2 = robot.getMotor("right wheel");


    int port = Integer.valueOf(robot.getCustomData());

    System.out.println( "Server -> starting on port " + port);
    Server httpController = new JavaController().listen(port);
    System.out.println(httpController.hashCode() + " : Server -> started on port " + port);


  


    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    // Motor motor = robot.getMotor("motorname");
    // DistanceSensor ds = robot.getDistanceSensor("dsname");
    // ds.enable(timeStep);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {

      // System.out.println("Timestep!");
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      // double val = ds.getValue();

      // Process sensor data here.

      // Enter here functions to send actuator commands, like:
      // motor.setPosition(10.0);

      // motor1.setPosition(60.0);
      // motor2.setPosition(60.0);

    }

    httpController.shutdown();

    // Enter here exit cleanup code.
  }
}
