# Web entry adapter (teammate's app, AGX side)

`s10_web_navigation.py` is the teammate's page-to-navigation adapter (`~/golai-web-entry/` on the AGX), with the
navigation team's patches of 2026-09-21: speed from `config/web_nav.json`, live route list, and automatic
localisation restore through `scripts/loc_keeper.py`. `s10_web_navigation.py.friend_20260921` is the file as
the teammate deployed it (roll back = copy it over and restart the adapter). Page and HTTP API live on 103
and are the teammate's; see their `S10_NAV_WEB_HANDOFF_ZH.md`.

`navteam_20260921.patch` is the navigation team's change as a unified diff against the teammate's file of 2026-09-21
(speed / climb speed / allow_stairs from `config/web_nav.json`, live route list, localisation restore only when there
is no pose at all, stairs-gait zones shifted by arc length when a route is joined mid-course). The teammate's own test
suite passes with it (48 of 49; the remaining one binds a non-local address and fails on any other machine) once two
expectations are updated: the arm speed comes from the config (default 0.3) and the join info carries `s_shift`.

The two `.py` files are the TEAMMATE'S code. Whether they go into the shared repository is the teammate's and the
project owner's call; the patch and this note are enough to reproduce our side.

Apply (from a directory that holds the teammate's file as `s10_web_navigation.py.friend_20260921`):

```bash
patch -o s10_web_navigation.py s10_web_navigation.py.friend_20260921 < navteam_20260921.patch
```

Regenerate after editing `s10_web_navigation.py`:

```bash
diff -u s10_web_navigation.py.friend_20260921 s10_web_navigation.py > navteam_20260921.patch
```

## page_103/

`page_103/*.patch` = our changes to the teammate's page on 103 (`/home/user/golai/`): an e-stop button that also stops
terminal-started runs, a start-from-waypoint select, and flat / stairs-gait speed inputs. Applied on 103 on 2026-09-21
(backup there: `backup_navteam_estop_20260921_1400/`). The teammate has edited the page since, so re-check the patch
against the live files before applying it again. `*.navteam` full copies stay out of the repository.
`test_navteam_handback.py` runs inside the teammate's test suite (it needs their adapter module), not on its own.
