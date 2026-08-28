// Copy this file to wifi_credentials.h and put the real values there.
//
// wifi_credentials.h is gitignored. This repository is PUBLIC, so a network
// password committed to it is a password published to the internet -- and
// rewriting history afterwards does not un-publish it, because clones and
// caches already have it. Keeping the real file out of git is the only
// version of this that actually works.
//
// The sketch includes wifi_credentials.h when it exists and falls back to
// placeholders when it does not, so a fresh clone still compiles.

#pragma once

#define VOLT_WIFI_NETWORKS \
  {"your-2.4GHz-ssid", "your-password"}, \
  /* {"second-network", "another-password"}, */
