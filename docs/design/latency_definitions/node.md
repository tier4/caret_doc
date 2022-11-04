# Node

Node latency is the time difference between when a callback takes ownership and when it abandons.

$$
l_{node} = t_{transfer} - t_{receive}
$$

In particular, if transfer of ownership is subscribe and receive of ownership is publish, it is defined as follows

$$
l_{node} = t_{pub} - t_{sub}
$$

See [Communication](./communication.md) for the reason ownership is used to define latency.

## Message context

A node receives a message, processes it, and then publishes it to a subsequent node.  
The dependency between the subscribed and published messages is used to define node latency.  
In the following sections explain the message dependency (message context), which is the concept of node latency.

For example, consider the following callback.

```c++
auto subsription_callback = [](&msg){
        msg_ = f(msg);
        pub.publish(msg_);
}
```

In this case, the received message is processed and published immediately.  
The dependency of the message at this time is described in chronological order and is expressed as follows.

![message context](../../imgs/message_context.drawio.png)

Here, the time difference between Subscribe and Publish can be calculated as node latency.  
In this way, node latency can be calculated when message dependencies are defined.
6
In the previous example, we presented a case where a node is subscribed and immediately published.
In actuality, the Message context can be quite complex, because node processing can be defined by the developer.

![complex message context](../../imgs/message_context_complex.drawio.png)

Each case is described below.

- Buffering, for example, is a buffer delay process.
- Multiple use is for moving average processing, for example. There are multiple candidates for node latency.
- Unused is a message that has not been published and has not been used. This is evaluated as a kind of message drop.
- Cross can occur in cases where messages are retrieved based on the timestamp of the message rather than the system time.

In either case, it is difficult to automatically determine the message context.

In some cases this can be covered by Configuration.
For more information, see [Configuration](. /... /configuration/index.md) for details.

If Configuration does not cover the situation, we are considering a mechanism to allow users to describe message dependencies as a future plan.
For details, please refer to [TILDE](. /software_architecture/tilde.md) for details.
