# CARET_analyze

caret_analyze is a package that loads trace data and architecture files and provides Python APIs for configuration and evaluation.

See [CARET analyze API document](https://tier4.github.io/CARET_analyze/) for the API of each class.

The following figure shows data flow in CARET_analyze.

![caret_analyze_architecture](../../imgs/architecture_caret_analyze.png)

Trace data is divided into two sections by CARET_analyze after loading trace data; Architecture and Runtime Data.

The architecture object includes descriptions of the target application's structure. This object can be reused unless the structure of the target application or names of the components is changed.

Runtime Data object has data sampled during the execution of the target application. The sampled data includes timestamps, whose values are different per execution, obtained from tracepoints.
Runtime data is combined with architecture and provided to developers via Python-API which is easy to evaluate.

Architecture object and Runtime Data object are implemented as Python classes.
The structure of their classes is designed based on the structure of ROS applications which are constructed of executors, nodes, callback functions, and topic messages. ROS-based structure makes CARET's API friendly for ROS users.

CARET_analyze is composed of several python packages.
Each python packages are as follows.

| python package | role                            |
| -------------- | ------------------------------- |
| architecture   | Load and configure Architecture |
| runtime        | Provide execution data          |
| value_objects  | collection of value objects     |
| plot           | Visualization helpers           |
| records        | implement records               |
| common         | common procedure                |
| infra          | import outer files              |

Role for each component is as follows.

```plantuml
package architecture {
  component Architecture
}
package runtime {
  component Application
}
package infra {
  component Lttng
  component Yaml
}
package value_objects {
  component NodeStructValue
  component NodeValue
}
package records {
  component Records
  component RecordsCppImpl
}

package plot {
  component Plot
}

interface "Runtime Data Provider" as Idata_provider
interface "Architecture Reader" as Iarchitecture_reader

interface "RecordsInterface" as Irecords


Lttng -- Idata_provider
Lttng -- Iarchitecture_reader
Yaml -- Iarchitecture_reader
Irecords -up- Records
Irecords -up- RecordsCppImpl
Application o-left- Architecture
Iarchitecture_reader <.down. Architecture : use
Idata_provider <.down. Application : use
Application <.. Plot : use
NodeValue <.. infra
architecture o-- NodeStructValue
Irecords <.. infra : use
```

Architecture object provides APIs to search node paths and define node latency as mentioned in [configuration chapter](../../configuration/index.md). The architecture object is reusable after it is saved as a YAML-based file called "architecture file".

Runtime Data object provides APIs to retrieve `pandas.DataFrame`-based objects including callback latency or communication. Users can analyze temporal aspects of their applications, with visualization, as they expect. APIs for visualization are also served by CARET_analyze which plays the main role to analyze trace data.

In the following sections, each package will be explained in more detail.

## architecture

In architecture, an instance is created with the following structure.
This allows access to the necessary information from the top-level Architecture class.

Purpose of Architecture:

- Define static information to be used in Analyze

<prettier-ignore-start>
!!! Info
      "Model" might be more appropriate than the name "Architecture".
      Architecture describes all the parameters related to scheduling, such as scheduling and core migration.
      Therefore, we're thinking that the architecture can be used for design based on scheduling theory.
<prettier-ignore-end>

```plantuml

class Architecture { }
class CallbackStructValue { }
class NodeStructValue { }
class PublisherStructValue { }
class SubscriptionStructValue { }
class ExecutorStructValue { }
class TimerStructValue { }
class CallbackGroupStructValue { }
class CommunicationStructValue { }
class PathStructValue { }
class NodePathStructValue { }
class VariablePassingStructValue { }

Architecture o-- NodeStructValue
Architecture o-- CommunicationStructValue
Architecture o-- PathStructValue
Architecture o-- ExecutorStructValue
PathStructValue o-d- CommunicationStructValue
PathStructValue o-d- NodePathStructValue
CommunicationStructValue o-- PublisherStructValue
CommunicationStructValue o-- SubscriptionStructValue
CommunicationStructValue o-l- NodeStructValue
NodeStructValue o-- CallbackGroupStructValue
NodeStructValue o-- VariablePassingStructValue
NodeStructValue o-l- NodePathStructValue
NodePathStructValue o-- PublisherStructValue
NodePathStructValue o-- SubscriptionStructValue
NodePathStructValue o-- CallbackStructValue
CallbackGroupStructValue o-d- CallbackStructValue
CallbackStructValue o-d- PublisherStructValue
CallbackStructValue o-d- TimerStructValue
CallbackStructValue o-d- SubscriptionStructValue
ExecutorStructValue o-- CallbackGroupStructValue
```

All information retrieved from the Architecture file is of type ValueObject,
which is suitable for interfacing information between other packages.

## runtime

The runtime, which also includes trace results, follows the architecture structure and adds a function to calculate the measurement results.

```plantuml

class Application { }
class CallbackBase { }
class Node { }
class Publisher { }
class Subscription { }
class Executor { }
class Timer { }
class CallbackGroup { }
class Communication { }
class Path { }
class NodePath { }
class VariablePassing { }

Application o-- Node
Application o-- Communication
Application o-- Path
Application o-- Executor
Path o-d- Communication
Path o-d- NodePath
Communication o-- Publisher
Communication o-- Subscription
Communication o-l- Node
Node o-- CallbackGroup
Node o-- VariablePassing
Node o-l- NodePath
NodePath o-- Publisher
NodePath o-- Subscription
NodePath o-- CallbackBase
CallbackGroup o-d- CallbackBase
CallbackBase o-d- Publisher
CallbackBase o-d- Timer
CallbackBase o-d- Subscription
Executor o-- CallbackGroup
```

There are classes that can calculate latency and classes as collections.
The following is a list of each class and the classes which can calculate latency.

| Class         | API                                                                                                   | has latency definition?                                      |
| ------------- | ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| Application   | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Application)   | No                                                           |
| Executor      | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Executor)      | No                                                           |
| Node          | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Node)          | No                                                           |
| Path          | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Path)          | Yes ([Definitions](../latency_definitions/path.md))          |
| NodePath      | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.NodePath)      | Yes ([Definitions](../latency_definitions/node.md))          |
| Communication | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Communication) | Yes ([Definitions](../latency_definitions/communication.md)) |
| Timer         | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Timer)         | Yes ([Definitions](../latency_definitions/timer.md))         |
| Subscription  | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Subscription)  | Yes ([Definitions](../latency_definitions/publisher.md))     |
| Callback      | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.CallbackBase)  | Yes ([Definitions](../latency_definitions/callback.md))      |

## value_objects

ValueObjects define classes with equivalence.
The Value class has the information for binding, and the StructValue class has the structure of multiple classes after binding.

## plot

There are classes associated with the display.
The visualization provided by CARET_analyze is based on bokeh and graphviz.

## records

In CARET, latency is calculated by joining process of tables uniquely defined.
The records package defines tables with their own join processing.

See also

- [Records](../processing_trace_data/records.md)

## common

Common package implements individual processes are described that can be handled as common in each package.

## infra

Infra package implements the process of reading from the outside.

It contains YAML and Lttng which implement ArchitectureReader/RuntimeDataProvider respectively.
