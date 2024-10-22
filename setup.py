import setuptools

setuptools.setup(
    name="EZMotion_MMS_1",
    description="Python Class for controlling EZMotion all in one servo motor",
    version="1.0.0",
    url="https://github.com/JnHold/EZMotion_MMS_1/",
    packages=setuptools.find_packages(),
    author="Jonathon Holder",
    author_email="jonathon.holder@griffithuni.edu.au",
    install_requires=["pyserial", "crccheck"],
    # keywords=['pip','linode','example']
)
