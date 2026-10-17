// npm run place-objects: places every saved detection again with the camera's numbers
// in place.ts (after a calibration), without asking the model (db.ts placeAgain).
import { Store } from "../store.ts";
import { DB_PATH } from "../import.ts";
import { placeAgain } from "./db.ts";
import { place } from "./place.ts";

const n = await placeAgain(new Store(DB_PATH), (d, pose) => place(d, pose));
console.log(`placed ${n} objects again`);
