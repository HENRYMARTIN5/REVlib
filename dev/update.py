import sys
import requests
import tempfile
import zipfile
from typing import NoReturn
import datetime
import argparse
import logging
import shutil

def main() -> NoReturn:
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("requests").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    parser = argparse.ArgumentParser(description="Tool to update source archive")
    parser.add_argument("-c", "--config", default="https://software-metadata.revrobotics.com/", help="Base URL for meta config file with a trailing slash (default: https://software-metadata.revrobotics.com/)")
    parser.add_argument("-y", "--year", default=str(datetime.datetime.now().year), help="Year to get REVLib for (default: current year)")
    args = parser.parse_args()
    
    logging.info("Getting config...")
    r = requests.get(f"{args.config}REVLib-{args.year}.json")
    r.raise_for_status()
    config = r.json()

    logging.debug(config)

    dep = config["javaDependencies"][0] # HACK: could change in the future but idc
    maven = f"{config["mavenUrls"][0]}{dep["groupId"].replace('.', '/')}/{dep["artifactId"]}/{dep["version"]}/{dep["artifactId"]}-{dep["version"]}-sources.jar"

    logging.info("Deleting old tree...")
    shutil.rmtree("src/main/java/com", ignore_errors=True)

    logging.info(f"Downloading REVLib for {args.year} from {maven}")
    r = requests.get(maven)
    with tempfile.TemporaryFile() as f:
        f.write(r.content)
        f.seek(0)
        with zipfile.ZipFile(f) as z:
            z.extractall("src/main/java")

    logging.info("Cleaning up...")
    shutil.rmtree("src/main/java/META-INF", ignore_errors=True)

    logging.info("Done!")

    sys.exit(0)

if __name__ == "__main__":
    main()