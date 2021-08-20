// File:          MyController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
import java.util.concurrent.locks.ReentrantLock;

import javax.jms.Connection;
import javax.jms.ConnectionFactory;
import javax.jms.Destination;
import javax.jms.JMSException;
import javax.jms.MessageConsumer;
import javax.jms.MessageProducer;
import javax.jms.Session;
import javax.jms.TextMessage;
import javax.json.bind.Jsonb;
import javax.json.bind.JsonbBuilder;
import javax.naming.Context;
import javax.naming.InitialContext;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Robot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Device;
import com.cyberbotics.webots.controller.LED;

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
  private static final double TURN_FACTOR = 35.1; // position to degrees

  static Motor motor1;
  static Motor motor2;
  static PositionSensor sensor1;
  static PositionSensor sensor2;
  static double wheelCircum;
  static double encoderUnit;
  static double distance = 0.0;
  static Camera camera;
  static LED led;
  static String robotName = "not defined";

  static ReentrantLock lock = new ReentrantLock();

  static Logger log = LoggerFactory.getLogger(JavaController.class);

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
    log.debug("{} - Current distance1 travelled -> {}", robotName, String.valueOf(distance));

  }  

  private static void forward(Robot robot, int timeStep, double worldDistance) {
    lock.lock();
    try {

      worldDistance = worldDistance * DRIVE_FACTOR;

      log.info("Action -> Forward " + worldDistance);

      robotStep(robot, timeStep);

      double offset1 = sensor1.getValue();
      double offset2 = sensor2.getValue();
      motor1.setPosition(offset1 + worldDistance);
      motor2.setPosition(offset2 + worldDistance);
      robotStep(robot, timeStep);


      int loop = 0;
      while ((sensor1.getValue() - offset1 < worldDistance && sensor2.getValue() - offset2 < worldDistance)
          || loop >= ACTION_LOOP_TIMEOUT) {

        robotStep(robot, timeStep);
        printDistanceTravelled(sensor1.getValue() - offset1);
        printDistanceTravelled(sensor1.getValue() - offset2);
        loop++;

      }
      log.info("{} - Action -> Done", robotName);

    } finally {
      lock.unlock();
    }
  }

  private static void right(Robot robot, int timeStep, double worldDegrees) {
    lock.lock();
    try {

      worldDegrees = worldDegrees / TURN_FACTOR;

      log.info("{} - Action -> Right {}", robotName, worldDegrees);

      robotStep(robot, timeStep);
      double offset1 = sensor1.getValue();
      double offset2 = sensor2.getValue();

      motor1.setPosition(offset1 + worldDegrees);
      motor2.setPosition(offset2 - worldDegrees);
      robotStep(robot, timeStep);

      int loop = 0;
      while ((sensor1.getValue() - offset1 < worldDegrees && sensor2.getValue() - offset2 < worldDegrees)
          || loop >= ACTION_LOOP_TIMEOUT) {

        robotStep(robot, timeStep);
        printDistanceTravelled(sensor1.getValue() - offset1);
        printDistanceTravelled(sensor1.getValue() - offset2);
        loop++;

      }
      log.info("{} - Action -> Done", robotName);

    } finally {
      lock.unlock();
    }
  }

  private static void left(Robot robot, int timeStep, double worldDegrees) {
    lock.lock();
    try {

      worldDegrees = worldDegrees / TURN_FACTOR;
      log.info("{} - Action -> Right ", robotName, worldDegrees);

      robotStep(robot, timeStep);
      double offset1 = sensor1.getValue();
      double offset2 = sensor2.getValue();

      motor1.setPosition(offset1 - worldDegrees);
      motor2.setPosition(offset2 + worldDegrees);
      robotStep(robot, timeStep);

      int loop = 0;
      while ((sensor1.getValue() - offset1 > worldDegrees && sensor2.getValue() - offset2 > worldDegrees)
          || loop >= ACTION_LOOP_TIMEOUT) {
        robotStep(robot, timeStep);
        printDistanceTravelled(sensor1.getValue() - offset1);
        printDistanceTravelled(sensor1.getValue() - offset2);
        loop++;

      }
      log.info("{} - Action -> Done", robotName);

    } finally {
      lock.unlock();
    }
  }

  private static void back(Robot robot, int timeStep, double worldDistance) {
    lock.lock();
    try {

      worldDistance = worldDistance * DRIVE_FACTOR;
      log.info("{} Action -> Back {}", robotName, worldDistance);

      robotStep(robot, timeStep);
      double offset1 = sensor1.getValue();
      double offset2 = sensor2.getValue();

      motor1.setPosition(offset1 - worldDistance);
      motor2.setPosition(offset2 - worldDistance);
      robotStep(robot, timeStep);

      int loop = 0;
      while ((sensor1.getValue() - offset1 > worldDistance && sensor2.getValue() - offset2 > worldDistance)
          || loop >= ACTION_LOOP_TIMEOUT) {
        robotStep(robot, timeStep);
        printDistanceTravelled(sensor1.getValue() - offset1);
        printDistanceTravelled(sensor1.getValue() - offset2);
        loop++;

      }
      log.info("{} Action -> Done", robotName);

    }

    finally {
      lock.unlock();
    }
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
    robotName = robot.getName();

    
    camera = robot.getCamera("camera");
    camera.enable(timeStep);
    
    led = robot.getLED("led");


    log.info("{} - Initializing robot", robotName);

    log.info("{} - REST Server starting on port -> {}",robotName, port);

    Undertow server = Undertow.builder().addHttpListener(port, "localhost")
        .setHandler(Handlers.pathTemplate().add("/forward/{length}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {

            double param = getParamAsDouble(exchange, "length");
            forward(robot, timeStep, param);
            exchange.getResponseSender().send("OK");

          }
        }).add("/right/{degrees}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {

            double worldDegrees = getParamAsDouble(exchange, "degrees");
            right(robot, timeStep, worldDegrees);
            exchange.getResponseSender().send("OK");

          }

        }).add("/left/{degrees}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {

            double worldDegrees = getParamAsDouble(exchange, "degrees");
            left(robot, timeStep, worldDegrees);
            exchange.getResponseSender().send("OK");

          }

        }).add("/back/{length}", new HttpHandler() {

          @Override
          public void handleRequest(HttpServerExchange exchange) throws Exception {

            double param = getParamAsDouble(exchange, "length");
            back(robot, timeStep, param);
            exchange.getResponseSender().send("OK");

          }

        }))

        .build();
    server.start();

    // Prepare JSM/AMQP Consumer

    Context context;
    MessageConsumer messageConsumer = null;
    MessageProducer replyProducer = null;
    Jsonb jsonb = JsonbBuilder.create();
    Session session = null;

    boolean amqConnectionEstablished = false;

    try {
      log.info("{} - Attempting to connect to AMQ Broker", robotName);
      context = new InitialContext();
      ConnectionFactory factory = (ConnectionFactory) context.lookup(robotName+"FactoryLookup");
      Destination queue = (Destination) context.lookup(robotName+"QueueLookup");
      Connection connection = factory.createConnection("admin", "admin");
      // connection.setExceptionListener(new MyExceptionListener());
      connection.start();
      session = connection.createSession(false, Session.AUTO_ACKNOWLEDGE);
      messageConsumer = session.createConsumer(queue);
      replyProducer = session.createProducer(null);
      amqConnectionEstablished = true;

    } catch (Exception e) {

      log.error("{} - Could not connect to AMQ Broker. Check your jndi.properties.",robotName,e);
      e.printStackTrace();
      led.set(0);
    }

    if (amqConnectionEstablished)
    {
      log.info("{} - AMQ Connected successfully", robotName);
      led.set(1);
    }  
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (lockRobotStep(robot, timeStep) != -1) {
      lockRobotStep(robot, timeStep);

      try {

        TextMessage receivedMessage = (TextMessage) messageConsumer.receive(1000L);

        if (receivedMessage == null) {
          log.debug("{} - Message not received within timeout", robotName);
        } else {
          log.info ("{} - Recieved message", robotName );

          RobotCommand robotCommand = jsonb.fromJson(receivedMessage.getText(), RobotCommand.class);

          log.info("{} - Received AMQ Command -> {}", robotName, robotCommand);

          if (robotCommand.getCommand().equals("forward"))
            forward(robot, timeStep, Double.valueOf(robotCommand.getParameter()));
          else if (robotCommand.getCommand().equals("back"))
            back(robot, timeStep, Double.valueOf(robotCommand.getParameter()));
          else if (robotCommand.getCommand().equals("left"))
            left(robot, timeStep, Double.valueOf(robotCommand.getParameter()));
          else if (robotCommand.getCommand().equals("right"))
            right(robot, timeStep, Double.valueOf(robotCommand.getParameter()));

          if (receivedMessage.getJMSReplyTo() != null)
            sendReply(replyProducer, jsonb, session, receivedMessage, robotCommand);

        }

      } catch (JMSException e1) {
        log.info("{} - Error proccessing AMQ Message", robotName);
        e1.printStackTrace();
      }

    

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error("{} - A general error occured", robotName, e);
    }
  }
  
  server.stop();


  // Enter here exit cleanup code.
  }

  private static void sendReply(MessageProducer replyProducer, Jsonb jsonb, Session session,
    TextMessage receivedMessage, RobotCommand robotCommand) throws JMSException {
    TextMessage response = session.createTextMessage();
    RobotCommand responseRobotCommand = new RobotCommand();
    responseRobotCommand.setCommand("OK");
    String answer = jsonb.toJson(responseRobotCommand);
    response.setText(answer);
    response.setJMSCorrelationID(receivedMessage.getJMSCorrelationID());
    log.info("{} - Replying with AMQ Command -> {}", robotName, responseRobotCommand);
    replyProducer.send(receivedMessage.getJMSReplyTo(), response);
  }

  private static int robotStep(Robot robot, int timeStep) {

    return robot.step(timeStep);

  }

  private static int lockRobotStep(Robot robot, int timeStep) {

    lock.lock();
    try {

      return robotStep(robot, timeStep);

    } finally {
      lock.unlock();
    }
  }

}
