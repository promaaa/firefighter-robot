# Documentation

| Topic | Document |
|---|---|
| Plan, milestones, workstreams and next tasks | [plan/README.md](plan/README.md) |
| Final design F2: layout, mass, print kit | [design.md](design.md) |
| Electrical design and drawings SOB-E | [hardware/electrical.md](hardware/electrical.md) |
| Water system: tanks, pump, nozzle, bench tests | [hardware/water.md](hardware/water.md) |
| Parts to order | [hardware/order_list.md](hardware/order_list.md) |
| Software architecture: nodes, messages, modes | [architecture.md](architecture.md) |

Each code folder has its own README with the commands: [robot](../robot/README.md), [legs](../legs/README.md), [actuators](../actuators/README.md), [aim](../aim/README.md), [vision](../vision/README.md), [sim](../sim/README.md), [cad](../cad/README.md).

## How we work

1. Pull before you start: `git pull`.
2. Work on a branch: `git switch -c <your-name>/<topic>`.
3. Run the tests of the folder you changed (each README gives the command).
4. Push the branch and open a pull request. One teammate reviews it before the merge.
5. Keep the documents current: change the README in the same pull request as the code or the CAD.

Status words in all documents: built, tested in simulation, measured on laptop, measured on board, planned.
