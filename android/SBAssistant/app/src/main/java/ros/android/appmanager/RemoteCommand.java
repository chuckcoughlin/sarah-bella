/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 *
 * @See: http://www.jcraft.com/jsch/examples/Exec.java.html
 * @See: http://www.jcraft.com/jsch/examples/Sudo.java.html
 */
package ros.android.appmanager;

import com.jcraft.jsch.Channel;
import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.Session;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Use ssh vis Java Secure Channel to execute a commands on the
 * robot system either as super-user or a specified user.
 *
 */
public class RemoteCommand {
    private final static String CLSS = "RemoteCommand";
    private static final int CONNECTION_TIMEOUT = 10000;  // msecs
    private final String username;
    private final String password;
    private final String hostname;  // Robot IP
    private final SBRemoteCommandListener commandListener;
    private final JSch jsch;

    /**
     * Constructor. Should not take any time.
     */
    public RemoteCommand(String host, String user, String pw, SBRemoteCommandListener listener) {
        this.hostname = host;
        this.username = user;
        this.password = pw;
        this.commandListener = listener;
        this.jsch = new JSch();
    }


    /**
     * Execute a command in a separate thread
     *
     * @param key a user-defined value returned in the callback to be used
     *            to identify the specific command which succeeded or failed
     * @param command to execute
     */

    public void execute(String key,String command) {
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Session session = jsch.getSession(username, hostname, 22);
                    session.setPassword(password);
                    session.setConfig("StrictHostKeyChecking", "no");
                    session.connect(CONNECTION_TIMEOUT);
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
                    commandListener.handleCommandCompletion(key,command,new String(tmp));
                }
                catch (Exception ex) {
                    commandListener.handleCommandError(key,command,ex.getLocalizedMessage());
                }
            }
        });
        thread.start();
    }

    /**
     * Execute a 'sudo' command in a separate thread. The supplied
     * username and password must belong to a 'sudoer'.
     *
     * @param key a user-defined value returned in the callback to be used
     *            to identify the specific command which succeeded or failed
     * @param command to execute
     */
    public void sudo(String key,String command) {
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
                    session.setConfig("StrictHostKeyChecking", "no");
                    session.connect(CONNECTION_TIMEOUT);
                    Channel channel = session.openChannel("exec");
                    ((ChannelExec)channel).setCommand("sudo -S -p '' "+command);
                    InputStream in = channel.getInputStream();
                    OutputStream out = channel.getOutputStream();
                    ((ChannelExec) channel).setErrStream(System.err);

                    channel.connect();
                    out.write((password+"\n").getBytes());
                    out.flush();

                    ByteArrayOutputStream baos = new ByteArrayOutputStream();
                    byte[] buffer = new byte[1024];
                    while (true) {
                        int read = 0;
                        while ((read = in.read(buffer, 0, buffer.length)) != -1) {
                            baos.write(buffer, 0, read);
                        }
                        baos.flush();

                        if (channel.isClosed()) {
                            System.out.println("exit-status: " + channel.getExitStatus());
                            break;
                        }
                        try {
                            Thread.sleep(500);
                        }
                        catch (InterruptedException ie) { }
                    }
                    channel.disconnect();
                    session.disconnect();
                    commandListener.handleCommandCompletion(key,command,new String(baos.toByteArray(), "UTF-8"));
                }
                catch (Exception ex) {
                    commandListener.handleCommandError(key,command,ex.getLocalizedMessage());
                }
            }
        });
        thread.start();
    }
}

