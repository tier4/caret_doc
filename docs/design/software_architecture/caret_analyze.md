# caret_analyze

`caret_analyze` is a set of packages that helps users to load trace data and architecture objects and provides Python APIs for configuration and evaluation.

See [CARET analyze API document](https://tier4.github.io/CARET_analyze/) for the definition of each class.

The following figure shows data flow in `caret_analyze`.

![caret_analyze_architecture](../../imgs/architecture_caret_analyze.png)

A set of trace data is divided into two sections after being loaded onto memory; architecture object and runtime data.

Architecture object includes descriptions of the target application's structure. This object can be reused unless the structure of the target application or names of the components is changed.

Runtime ata object has data sampled during the execution of the target application. The sampled data includes timestamps, whose values are different per execution, obtained from tracepoints.
Runtime data is combined with architecture and provided to developers via Python-API which is easy to evaluate.

Architecture object and runtime data are instantiated from respective Python classes.
Structure of their classes is designed based on that of ROS applications which are constructed of executors, nodes, callback functions, and topic messages. ROS-based structure makes CARET's API friendly for ROS users.

`caret_analyze` is composed of several python packages.
Each python packages are as follows.

| python package | role                            |
| -------------- | ------------------------------- |
| `architecture`   | Load and configure an architecture object |
| `runtime`        | Provide execution data          |
| `value_objects`  | Collection of value objects     |
| `plot`           | Visualization helpers           |
| `records`        | Implementation of records               |
| `common`         | Common or helper functions                 |
| `infra`          | Import external files              |

Each of the packages has interrelation with other as shown in the following diagram.

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

Architecture object provides APIs to search node paths and define intra-node data paths as mentioned in [configuration chapter](../../configuration/index.md). The architecture object is reusable after it is saved as a YAML-based file called "architecture file".

Runtime data provides APIs to retrieve `pandas.DataFrame`-based objects including callback latency or communication. Users can analyze temporal aspects of their applications, with visualization, as they expect. APIs for visualization are also served by `caret_analyze` which plays the main role to visualize trace data.

In the following sections, each package will be explained in more detail.

## `architecture`

The purpose of `architecture` to define static information to be used in visualization.

`architecture` package serves classes which embody architecture object. Architecture object has one or more sub components.
There are several types of components; executor, node, callback and topic.
CARET serves a class to each type of component and manages them in `architecture` package.

A target application, which is represented with `architecture` class, has several sub sub components. A `architecture`-based object has several types of sub components as well.

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

All sub objects retrieved from the Architecture object are constructed from `ValueObject`,
which is suitable for interfacing data with other packages.

## `runtime`

`runtime` a package to hold trace data, whose object has similar data structure to that of Architecture object.
Objects instantiated from `runtime` package have function to return measured data like frequency or latency.

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

The following is a list of each class. Some of them are able to return measured data and they are remarked.

| Class         | API                                                                                                   | has measured data definition?                                                |
| ------------- | ----------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| Application   | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Application)   | No                                                                     |
| Executor      | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Executor)      | No                                                                     |
| Node          | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Node)          | No                                                                     |
| Path          | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Path)          | Yes ([Definitions](../event_and_latency_definitions/path.md))          |
| NodePath      | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.NodePath)      | Yes ([Definitions](../event_and_latency_definitions/node.md))          |
| Communication | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Communication) | Yes ([Definitions](../event_and_latency_definitions/communication.md)) |
| Timer         | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Timer)         | Yes ([Definitions](../event_and_latency_definitions/timer.md))         |
| Subscription  | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.Subscription)  | Yes ([Definitions](../event_and_latency_definitions/publisher.md))     |
| Callback      | [API list](https://tier4.github.io/CARET_analyze/latest/runtime/#caret_analyze.runtime.CallbackBase)  | Yes ([Definitions](../event_and_latency_definitions/callback.md))      |

## `value_objects`

`value_objects` define classes with equivalence.
The Value class has the information for binding, and the StructValue class has the structure of multiple classes after binding.

## `plot`

`plot` package has classes associated with visualization.
The visualization methods provided by `caret_analyze` depends on `bokeh` and `graphviz`.

## records

latency is calculated by joining process of tables uniquely defined.
`records` package serves functions to make the tables with their own join processing.

See also

- [Records](../processing_trace_data/records.md)

## common

Common package implements individual processes are described that can be handled as common in each package.

## infra

`infra` package serves readers for an architecture object and trace data.

It contains YAML and LTTng modules which implement `ArchitectureReader`/`RuntimeDataProvider` respectively.
