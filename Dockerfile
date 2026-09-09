# ESP-IDF v6.1, pinned to the published linux/amd64 image.
FROM espressif/idf:v6.1@sha256:f81b6e077e6af93c312181cd7d4e7936cfbafc5c7f77cbacba8fdd34994f218a

WORKDIR /project
COPY . .
