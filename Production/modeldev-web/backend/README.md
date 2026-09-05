# ModelDev Web backend

This is the local single-user API for the static UI in `../static`.
It deliberately exposes only the server-side HTV2 and BGM whitelist. A browser
cannot submit an executable path, command line, or result directory.

## Start

From the repository root:

```powershell
python -m pip install -r .\Production\modeldev-web\backend\requirements.txt
python -m uvicorn backend.app:app --app-dir .\Production\modeldev-web --host 127.0.0.1 --port 8000
```

Open `http://127.0.0.1:8000/`. By default, completed and in-progress runs are
stored under `backend/data/runs/<run_id>`. `MODELDEV_RUN_ROOT` can select a
service-owned data directory, and `MODELDEV_RUN_TIMEOUT_SECONDS` can set a
timeout from 1 to 3600 seconds.

## API

- `GET /api/models`
- `GET /api/model-catalog`
- `POST /api/runs` with `model_id` and optional `name`/`parameters`/`modules`
- `GET /api/runs?limit=50&model_id=HTV2&status=succeeded`
- `GET /api/runs/{run_id}`
- `POST /api/runs/{run_id}/cancel`
- `GET /api/runs/{run_id}/result`
- `GET /api/runs/{run_id}/export?format=json|csv|raw`
- `POST /api/comparisons` with two to four unique `run_ids`

`GET /api/models` remains the runnable-model configuration contract used by
`POST /api/runs`. `GET /api/model-catalog` is a separate display-oriented
library grouped into the stable categories `vehicle`, `environment`,
`guidance`, `control`, `sensor`, and `recorder`. Each catalog entry returns:

- identity and display metadata: `id`, `name`, `description`, `kind`, and the
  source-declared `version` (`null` when this repository has no version claim);
- `capabilities` and `contracts` backed by the listed source evidence;
- repository-relative `source_path` and `evidence` only—never an absolute host
  path or executable path;
- `status`, `status_label`, `run_model_id`, `source_present`, and `runnable`.

Only HTV2 and BGM catalog entries map to the trusted runnable registry. Their
status is `runnable` only when the registered executable exists, otherwise it
is `build_required`. Other whole-vehicle sources are `not_integrated`; source
components such as guidance, control, IMU, atmosphere, and file writers also
use `not_integrated` and are labelled `随整机运行`. Catalog-only ids are not
accepted by `POST /api/runs`.

The service runs one simulation subprocess at a time. Every process uses its
own working directory, has `shell=False`, captures stdout, and is terminated on
timeout or cancellation. The result endpoint returns at most 5000 trajectory
samples while preserving the first and last sample.

Each model advertises its common numeric parameters and first-class
`guidance`/`control` module slots from `GET /api/models`. A run may select a
registered option and override only that option's numeric parameters:

```json
{
  "model_id": "HTV2",
  "modules": {
    "guidance": {
      "id": "phase_standard",
      "parameters": {"navigation_constant": 5}
    },
    "control": {
      "id": "p6dof_p",
      "parameters": {"gain_scale": 0.8, "rudder_limit_deg": 30}
    }
  }
}
```

Omitted slots use server-side defaults. The server rejects unknown slots,
options, inactive-option parameters, and invalid numeric values. Executable
paths, CLI flags, option CLI values, commands, and result paths remain private
server configuration; the process still receives only registry-built argv with
`shell=False`.

Every run directory contains an atomically replaced manifest-v2 `run.json`, an
appended `run.log`, and the model artifact. The normalized module selections
are included in history and comparison records. Existing v1 manifests reload
with an empty module snapshot rather than being mislabeled with today's
defaults. A run is marked successful only after its result file passes the
versioned parser contract.
