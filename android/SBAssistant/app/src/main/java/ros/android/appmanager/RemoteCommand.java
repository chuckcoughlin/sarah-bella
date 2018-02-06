/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */
package ros.android.appmanager;

import android.util.Log;

import com.jcraft.jsch.Channel;
import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.Session;
import com.jcraft.jsch.UserInfo;

import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Date;
import java.util.List;

import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

/**
 * Use ssh vis Java Secure Channel to execute a commands on the
 * robot system either as super-user or a specified user.
 *
 */
public class RemoteCommand {
    private final static String CLSS = "RemoteCommand";
    private final String username;
    private final String password;
    private final String hostname;  // Robot IP
    private final SBRobotConnectionErrorListener errorListener;
    private final JSch jsch;

    /**
     * Constructor. Should not take any time.
     */
    public RemoteCommand(String host, String user, String pw, SBRobotConnectionErrorListener listener) {
        this.hostname = host;
        this.username = user;
        this.password = pw;
        this.errorListener = listener;
        this.jsch = new JSch();
    }


    /**
     * Execute a command in a separate thread
     *
     * @param command to execute
     */

    public void execute(String command) {
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Session session = jsch.getSession(username, hostname, 22);
                    session.setPassword(password);
                    session.connect();
                    Channel channel = session.openChannel("exec");
                    ((ChannelExec) channel).setCommand(command);
                    InputStream in = channel.getInputStream();
                    OutputStream out = channel.getOutputStream();
                    ((ChannelExec) channel).setErrStream(System.err);

                    channel.connect();

                    byte[] tmp = new byte[1024];
                    while (true) {
                        while (in.available() > 0) {
                            int i = in.read(tmp, 0, 1024);
                            if (i < 0) break;
                            System.out.print(new String(tmp, 0, i));
                        }
                        if (channel.isClosed()) {
                            System.out.println("exit-status: " + channel.getExitStatus());
                            break;
                        }
                        try {
                            Thread.sleep(1000);
                        } catch (Exception ee) {
                        }
                    }
                    channel.disconnect();
                    session.disconnect();
                } catch (Exception ex) {
                    errorListener.handleConnectionError(String.format("Error executing: %s (%s)", command, ex.getLocalizedMessage()));
                }
            }
        });
        thread.start();
    }

    /**
     * Execute a 'sudo' command in a separate thread. The supplied
     * username and password must belong to a 'sudoer'.
     *
     * @param command to execute
     */
    public void sudo(String command) {
        // from man sudo
        //   -S  The -S (stdin) option causes sudo to read the password from the
        //       standard input instead of the terminal device.
        //   -p  The -p (prompt) option allows you to override the default
        //       password prompt and use a custom one.
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Session session = jsch.getSession(username, hostname, 22);
                    session.setPassword(password);
                    session.connect();
                    Channel channel = session.openChannel("exec");
                    ((ChannelExec)channel).setCommand("sudo -S -p '' "+command);
                    InputStream in = channel.getInputStream();
                    OutputStream out = channel.getOutputStream();
                    ((ChannelExec) channel).setErrStream(System.err);

                    channel.connect();
                    out.write((password+"\n").getBytes());
                    out.flush();

                    byte[] tmp = new byte[1024];
                    while (true) {
                        while (in.available() > 0) {
                            int i = in.read(tmp, 0, 1024);
                            if (i < 0) break;
                            System.out.print(new String(tmp, 0, i));
                        }
                        if (channel.isClosed()) {
                            System.out.println("exit-status: " + channel.getExitStatus());
                            break;
                        }
                        try {
                            Thread.sleep(1000);
                        } catch (Exception ee) {
                        }
                    }
                    channel.disconnect();
                    session.disconnect();
                } catch (Exception ex) {
                    errorListener.handleConnectionError(String.format("Error executing: %s (%s)", command, ex.getLocalizedMessage()));
                }
            }
        });
        thread.start();
    }
}

