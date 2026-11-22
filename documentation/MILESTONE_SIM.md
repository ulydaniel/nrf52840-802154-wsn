# Milestone Simulator (assignment §11)

**Script:** `Host/milestone_sim.py`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §7  
**Code mirrored:** application rules from `src/main.c` (OZ header, FSM, allocation, join, DATA, reliability)

This is a **host-side** dry-run. It does not use Zephyr or the nRF52840 radio. Use it to confirm each milestone’s expected result before (or alongside) a hardware demo.

---

## Run

```text
python Host/milestone_sim.py              # all M1–M7
python Host/milestone_sim.py --milestone 6
```

Exit code `0` = all selected milestones PASS.

---

## What each milestone proves

| M | Name | Expected result in simulator |
|---|------|------------------------------|
| 1 | Packet format | 17-byte LE header; round-trip; bad `"OZ"` / wrong version rejected; types 1–8 |
| 2 | Discovery | Joiner broadcasts `PROBE`; root replies `HEARTBEAT`; candidate recorded |
| 3 | Parent selection | Policy: lowest hop → strongest RSSI → lowest address; live pick prefers root hop 0 |
| 4 | Address assignment | Root issues unique `1:1`, `1:2`, `1:3` via `JOIN_RESPONSE` |
| 5 | Registration | Joiner → `REGISTERED`, applies address, sends `JOIN_ACK`; root marks **ACTIVE** |
| 6 | Data collection | Child → relay → root; origin `source_id` preserved; root collects sequences |
| 7 | Reliability | Duplicate nonce → no second alloc; lost request → retransmit; pending release + reuse; bad packet reject; DATA gap log; max retries → `PROBING` |

---

## Relation to hardware

| Layer | Simulator | Boards |
|-------|-----------|--------|
| OZ serialize / handlers / FSM / table | Exercised | Same logic in `main.c` |
| IEEE 802.15.4 MAC / RSSI / RTT logs | Abstracted | Required for checklist §8–§9 evidence |
| Wireshark / sniffer | N/A | Use `Host/nrf802154_serial_sniffer.py` |

Passing the simulator means the **protocol logic** matches the milestone contracts. Serial logs from one root + ≥2 joining boards are still required for the graded demonstration.
