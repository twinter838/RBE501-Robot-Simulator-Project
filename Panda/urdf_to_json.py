from datetime import datetime
import json
from pathlib import Path

from timor.Robot import PinRobot
from timor.Module import ModuleAssembly
from timor.utilities.jsonable import compress_json_vectors
from timor.utilities.schema import DEFAULT_DATE_FORMAT


def main():
    package_dir = Path(__file__).parent
    panda_urdf = package_dir.joinpath("panda.urdf")
    panda_assembly = ModuleAssembly.from_monolithic_robot(PinRobot.from_urdf(panda_urdf, package_dir),
                                                          urdf=panda_urdf)
    # Fix author, date, etc. such that deterministic CI
    db_dict = panda_assembly.db.to_json_data()
    for m in db_dict:
        m["header"]["date"] = datetime(2023, 3, 1).strftime(DEFAULT_DATE_FORMAT)
        m["header"]["author"] = ["Matthias Mayer", "Jonathan Kuelz", "Panda Author"]
        m["header"]["email"] = ["matthias.mayer@tum.de"]
        m["header"]["affiliation"] = ["TU Munich"]

    db_string = compress_json_vectors(json.dumps(db_dict, indent=2))
    with open(package_dir.joinpath("modules.json"), "w") as f:
        f.write(db_string)


if __name__ == '__main__':
    main()
