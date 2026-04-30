
import json, pathlib

path = pathlib.Path(r'C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json')
state = json.loads(path.read_text())
state['operational_error_count'] = 0
state['last_operational_error'] = None
path.write_text(json.dumps(state, indent=2))
print(f'operational_error_count reset to 0')
print(f'fail_safe_active: {state.get("fail_safe_active")}')
print(f'auton_mode: {state.get("auton_mode")}')
