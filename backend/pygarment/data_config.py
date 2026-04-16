"""
    Vendored & stripped data_config from GarmentCode.

    The Properties class manages parameters & stats in various parts of the
    system. Stripped: removed psutil, warp, and platform imports from
    add_sys_info() and stats_summary().  These are only used for dataset
    generation logging, not for mesh generation.

    Original: pygarment/data_config.py (GarmentCode)
"""

from datetime import timedelta
import json
import yaml
from numbers import Number
import traceback
import sys
from pathlib import Path
import numpy as np


# --- Nice dumping of floats ---
def float_representer(dumper, data):
    if data != data or (data == 0.0 and data == 1.0):
        value = '.nan'
    elif data == dumper.inf_value:
        value = '.inf'
    elif data == -dumper.inf_value:
        value = '-.inf'
    else:
        value = f'{data:.3g}'
        if '.' not in value or 'e' in value:
            value = f'{int(data):d}.0'

    return dumper.represent_scalar('tag:yaml.org,2002:float', value)

yaml.add_representer(float, float_representer)


# --- Main class ----
class Properties():
    """Keeps, loads, and saves configuration & statistic information.
    Supports gets & sets as a dictionary.
    """
    def __init__(self, filename="", clean_stats=False):
        self.properties = {}
        self.properties_on_load = {}

        if filename:
            self.properties = self._from_file(filename)
            self.properties_on_load = self._from_file(filename)
            if clean_stats:
                self.clean_stats(self.properties)

    # ---- Base utils ----
    def has(self, key):
        return key in self.properties

    def serialize(self, filename, backup=None):
        """Log current props to file."""
        try:
            extention = Path(filename).suffix.lower()
            if extention == '.json':
                with open(filename, 'w') as f_json:
                    json.dump(self.properties, f_json, indent=2, sort_keys=True)
            elif extention == '.yaml':
                with open(filename, 'w') as f:
                    yaml.dump(
                        self.properties, f,
                        default_flow_style=False, sort_keys=False
                    )
            else:
                raise ValueError(
                    f'{self.__class__.__name__}::ERROR::Unsupported file type: {extention}'
                )
        except Exception:
            traceback.print_exception(*sys.exc_info())
            if backup is not None:
                backup.serialize(filename)
            else:
                with open(filename, 'w') as f_json:
                    json.dump(self.properties_on_load, f_json, indent=2, sort_keys=True)
            raise RuntimeError('Error saving properties. Backup saved instead.')

    def merge(self, filename="", clean_stats=False, re_write=True,
              adding_tag='added'):
        new_props = self._from_file(filename)
        if clean_stats:
            self.clean_stats(new_props)
        self._recursive_dict_update(self.properties, new_props, re_write, adding_tag)

    # --- Specialised utils ---
    def is_fail(self, dataname):
        _, fails_list = self.count_fails()
        return dataname in fails_list

    def count_fails(self, log=False):
        fails = []
        for section_key in self.properties:
            section = self.properties[section_key]
            section_fails = []
            if isinstance(section, dict) and 'stats' in section and ('fails' in section['stats']):
                if isinstance(section['stats']['fails'], dict):
                    for key in section['stats']['fails']:
                        section_fails += section['stats']['fails'][key]
                elif isinstance(section['stats']['fails'], list):
                    section_fails += section['stats']['fails']
                if log:
                    section['stats']['fails_count'] = len(list(set(section_fails)))
                fails += section_fails
        fails = list(set(fails))
        return len(fails), fails

    def add_fail(self, section_name, fail_type, info):
        section = self.properties[section_name]
        if 'fails' not in section['stats']:
            section['stats']['fails'] = {}
        try:
            section['stats']['fails'][fail_type].append(info)
        except KeyError:
            section['stats']['fails'][fail_type] = [info]

    # ---------- Properties updates ---------------
    def set_basic(self, **kwconfig):
        for key, value in kwconfig.items():
            self.properties[key] = value

    def set_section_config(self, section, **kwconfig):
        if section not in self.properties:
            self.properties[section] = {'config': kwconfig, 'stats': {}}
            return
        for key, value in kwconfig.items():
            self.properties[section]['config'][key] = value

    def set_section_stats(self, section, **kwstats):
        if section not in self.properties:
            self.properties[section] = {'config': {}, 'stats': kwstats}
            return
        for key, value in kwstats.items():
            self.properties[section]['stats'][key] = value

    def clean_stats(self, properties):
        for _, value in properties.items():
            if isinstance(value, dict) and 'stats' in value:
                value['stats'] = {}

    def summarize_stats(self, key, log_sum=False, log_avg=False,
                        log_median=False, log_80=False, log_95=False,
                        log_min=False, log_max=False, as_time=False):
        updated = False
        for section in self.properties.values():
            if isinstance(section, dict) and 'stats' in section:
                if key in section['stats']:
                    stats_values = section['stats'][key]
                    if isinstance(stats_values, dict):
                        stats_values = list(stats_values.values())
                    if isinstance(stats_values, list) and len(stats_values) > 0 and isinstance(stats_values[0], Number):
                        if log_sum:
                            section['stats'][key + "_sum"] = (
                                str(timedelta(seconds=sum(stats_values))) if as_time else sum(stats_values))
                            updated = True
                        if log_avg:
                            avg = sum(stats_values) / len(stats_values)
                            section['stats'][key + "_avg"] = str(timedelta(seconds=avg)) if as_time else avg
                            updated = True
                        if log_median:
                            section['stats'][key + "_med"] = float(np.percentile(stats_values, 50))
                            updated = True
                        if log_min:
                            section['stats'][key + "_min"] = min(stats_values)
                            updated = True
                        if log_max:
                            section['stats'][key + "_max"] = max(stats_values)
                            updated = True
        return updated

    def add_sys_info(self):
        """Stubbed — original uses psutil and warp for system info."""
        import platform
        self.properties['system_info'] = {
            'platform': platform.system(),
            'platform-release': platform.release(),
            'architecture': platform.machine(),
            'processor': platform.processor(),
        }

    # ---- Private utils ----
    def _from_file(self, filename):
        extention = Path(filename).suffix.lower()
        if extention == '.json':
            with open(filename, 'r') as f_json:
                return json.load(f_json)
        elif extention == '.yaml':
            with open(filename, 'r') as f:
                return yaml.safe_load(f)
        else:
            raise ValueError(
                f'{self.__class__.__name__}::ERROR::Unsupported file type: {extention}'
            )

    def _recursive_dict_update(self, in_dict, new_dict, re_write=True,
                               adding_tag='added', in_stats=False):
        if not isinstance(new_dict, dict):
            in_dict = new_dict
            return
        for new_key in new_dict:
            if new_key in in_dict and isinstance(in_dict[new_key], dict):
                self._recursive_dict_update(
                    in_dict[new_key], new_dict[new_key],
                    re_write, adding_tag,
                    (in_stats or new_key == 'stats'))
            elif not re_write and new_key in in_dict and in_dict[new_key] != new_dict[new_key]:
                if in_stats and isinstance(in_dict[new_key], list):
                    in_dict[new_key] = in_dict[new_key] + new_dict[new_key]
                else:
                    adding_name = new_key + '_' + adding_tag
                    while adding_name in in_dict:
                        adding_name = adding_name + '_added'
                    in_dict[adding_name] = new_dict[new_key]
            else:
                in_dict[new_key] = new_dict[new_key]

    def __getitem__(self, key):
        return self.properties[key]

    def __setitem__(self, key, value):
        self.properties[key] = value

    def __contains__(self, key):
        return key in self.properties

    def __str__(self):
        return str(self.properties)
