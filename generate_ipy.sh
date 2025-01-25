cd py-bindings
rm -rf stubs
mkdir stubs
stubgen -p ompl -o stubs
cd -
