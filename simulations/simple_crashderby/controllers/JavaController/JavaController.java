// File:          MyController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Robot;

import io.undertow.Handlers;
import io.undertow.Undertow;
import io.undertow.server.HttpHandler;
import io.undertow.server.HttpServerExchange;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class JavaController {

  private static final int ACTION_LOOP_TIMEOUT = 200; // Number of loops to execute until an action will timeout
  private static final String NAME_RIGHT_WHEEL_MOTOR = "right wheel motor";
  private static final String NAME_LEFT_WHEEL_MOTOR = "left wheel motor";
  private static final double WHEEL_DIAMETER = 4.1;
  private static final double DEFAULT_SPEED = 6.28;
  private static final double DRIVE_FACTOR = 12.2; // positions to meters
  private static final double TURN_FACTOR = 30; // position to degrees

  static Motor motor1;
  static Motor motor2;
  static PositionSensor sensor1;
  static PositionSensor sensor2;
  static double wheelCircum;
  static double encoderUnit;
  static double distance = 0.0;

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node

  private static double getParamAsDouble(HttpServerExchange exchange, String paramName) {
    if (exchange.getQueryParameters().containsKey(paramName))
      return Double.valueOf(exchange.getQueryParameters().get(paramName).getFirst());
    else
      return 0.0;

  }

  private static void printDistanceTravelled(double distance) {
    System.out.println("Current distance1 travelled -> " + String.valueOf(distance));

  }

  private static void setMotorPositionWithOffset(Motor motor1, Motor motor2) {
    System.out.println("Current distance1 travelled -> " + String.valueOf(distance));

  }

  public static void main(String[] args) {

    // create the Robot instance.
    Robot robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    motor1 = robot.getMotor(NAME_LEFT_WHEEL_MOTOR);
    motor2 = robot.getMotor(NAME_RIGHT_WHEEL_MOTOR);

    motor1.setVelocity(DEFAULT_SPEED);
    motor2.setVelocity(DEFAULT_SPEED);

    sensor1 = motor1.getPositionSensor();
    sensor1.enable(timeStep);
    sensor2 = motor2.getPositionSensor();
    sensor2.enable(timeStep);

    int port = Integer.valueOf(robot.getCustomData());

    System.out.println("Server -> starting on port " + port);

    Undertow server = Undertow.builder().addHttpListener(port, "localhost")
        .setHandler(Handlers.pathTemplate().add("/forward/{length}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {

            double worldDistance = getParamAsDouble(exchange, "length") * DRIVE_FACTOR;
            System.out.println("Action -> Forward " + worldDistance);

            double offset1 = sensor1.getValue();
            double offset2 = sensor2.getValue();

            motor1.setPosition(offset1 + worldDistance);
            motor2.setPosition(offset2 + worldDistance);

            int loop = 0;
            while ((sensor1.getValue() - offset1 < worldDistance && sensor2.getValue() - offset2 < worldDistance)
                || loop >= ACTION_LOOP_TIMEOUT) {

              printDistanceTravelled(sensor1.getValue() - offset1);
              printDistanceTravelled(sensor1.getValue() - offset2);
              loop++;
            }
            System.out.println("Action -> Done");
            exchange.getResponseSender().send("OK");

          }
        }).add("/right/{degrees}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {
            double worldDegrees = getParamAsDouble(exchange, "degrees") / TURN_FACTOR;
            System.out.println("Action -> Right " + worldDegrees);

            double offset1 = sensor1.getValue();
            double offset2 = sensor2.getValue();

            motor1.setPosition(offset1 + worldDegrees);
            motor2.setPosition(offset2 - worldDegrees);

            int loop = 0;
            while ((sensor1.getValue() - offset1 < worldDegrees && sensor2.getValue() - offset2 < worldDegrees)
                || loop >= ACTION_LOOP_TIMEOUT) {
              printDistanceTravelled(sensor1.getValue() - offset1);
              printDistanceTravelled(sensor1.getValue() - offset2);
              loop++;

            }
            System.out.println("Action -> Done");
            exchange.getResponseSender().send("OK");

          }
        }).add("/left/{degrees}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {
            double worldDegrees = getParamAsDouble(exchange, "degrees") / TURN_FACTOR;
            System.out.println("Action -> Left " + worldDegrees);

            double offset1 = sensor1.getValue();
            double offset2 = sensor2.getValue();

            motor1.setPosition(offset1 - worldDegrees);
            motor2.setPosition(offset2 + worldDegrees);

            int loop = 0;
            while ((sensor1.getValue() - offset1 > worldDegrees && sensor2.getValue() - offset2 > worldDegrees)
                || loop >= ACTION_LOOP_TIMEOUT) {
              printDistanceTravelled(sensor1.getValue() - offset1);
              printDistanceTravelled(sensor1.getValue() - offset2);
              loop++;

            }
            System.out.println("Action -> Done");
            exchange.getResponseSender().send("OK");

          }
        }).add("/back/{length}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {
            double worldDistance = getParamAsDouble(exchange, "length") * DRIVE_FACTOR;
            System.out.println("Action -> Back " + worldDistance);

            double offset1 = sensor1.getValue();
            double offset2 = sensor2.getValue();

            motor1.setPosition(offset1 - worldDistance);
            motor2.setPosition(offset2 - worldDistance);

            int loop = 0;
            while ((sensor1.getValue() - offset1 > worldDistance && sensor2.getValue() - offset2 > worldDistance)
                || loop >= ACTION_LOOP_TIMEOUT) {
              printDistanceTravelled(sensor1.getValue() - offset1);
              printDistanceTravelled(sensor1.getValue() - offset2);
              loop++;

            }
            System.out.println("Action -> Done");
            exchange.getResponseSender().send("OK");

          }
        }))

        .build();
    server.start();

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {

    }
    server.stop();

    // Enter here exit cleanup code.
  }
}
