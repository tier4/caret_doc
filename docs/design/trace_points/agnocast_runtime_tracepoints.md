### Relationships of each Agnocast runtime trace points

```mermaid
erDiagram
  agnocast_publish {
    address publisher_handle
    int64_t entry_id
  }

  agnocast_create_callable {
    address callable
    int64_t entry_id
    uint64_t pid_callback_info_id
  }

  agnocast_create_timer_callable {
    address callable
    address timer_handle
  }

  agnocast_callable_start {
    address callable
  }

  agnocast_callable_end {
    address callable
  }

  agnocast_take {
    address subscription_handle
    address message
    int64_t entry_id
  }

  agnocast_publish ||--o{ agnocast_create_callable : entry_id
  agnocast_publish ||--o{ agnocast_take : entry_id
  agnocast_create_callable ||--|| agnocast_callable_start : callable
  agnocast_create_timer_callable ||--|| agnocast_callable_start : callable
  agnocast_callable_start ||--|| agnocast_callable_end : callable
```

In Agnocast, message passing uses `entry_id` to associate a publish event with the corresponding subscription callback execution.
`agnocast_publish` is linked to `agnocast_create_callable` and `agnocast_take` via `entry_id`.
The callable lifecycle (`agnocast_callable_start` / `agnocast_callable_end`) is tracked via `callable` address.

### Trace point definition

#### agnocast:agnocast_publish

[Built-in tracepoints]

Sampled items

- void \* publisher_handle
- int64_t entry_id

---

#### agnocast:agnocast_create_callable

[Built-in tracepoints]

Sampled items

- void \* callable
- int64_t entry_id
- uint64_t pid_callback_info_id

<prettier-ignore-start>
!!!Note
    In older versions, `pid_callback_info_id` may be recorded as `pid_ciid`.
<prettier-ignore-end>

---

#### agnocast:agnocast_create_timer_callable

[Built-in tracepoints]

Sampled items

- void \* callable
- void \* timer_handle

---

#### agnocast:agnocast_callable_start

[Built-in tracepoints]

Sampled items

- void \* callable

---

#### agnocast:agnocast_callable_end

[Built-in tracepoints]

Sampled items

- void \* callable

---

#### agnocast:agnocast_take

[Built-in tracepoints]

Sampled items

- void \* subscription_handle
- void \* message
- int64_t entry_id
