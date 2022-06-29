```plantuml
class Application {
    + get_callback(callback_name: int) Callback [[[https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.application.Application.get_callback]]]
}
class Callback
class Node
class Publisher
class Subscription
class Executor
class Timer
class CallbackGroup
class VariablePassing
class Communication

Application o-- Node
Application o-- Communication
Communication o-- Publisher
Communication o-- Subscription
Node o-- CallbackGroup
Node o-- VariablePassing
CallbackGroup o-- Callback
Callback o-- Publisher
Callback o-- Timer
Callback o-- Subscription
Executor o-- CallbackGroup
```
