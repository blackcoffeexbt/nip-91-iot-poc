float simulateTemperature(unsigned long timestamp) {
  static bool seeded = false;
  if (!seeded) {
    randomSeed(analogRead(0));  // Use an unconnected analog pin to seed randomness
    seeded = true;
  }

  struct tm *timeinfo;
  timeinfo = gmtime((const time_t *)&timestamp);

  // Extract the hour from the timestamp
  int hour = timeinfo->tm_hour;

  // Use a basic sine wave model to simulate diurnal temperature variation.
  // This won't be completely accurate but gives a decent representation for our purposes.
  float baselineTemperature = 11.5;  // mid-point between 8 and 15°C
  float amplitude = 3.5;  // (15-8)/2 = 3.5°C
  float simulatedTemperature = baselineTemperature + amplitude * sin((hour-3) * M_PI / 12);  // -3 to peak at around 15°C at 12 noon

  // Add randomness to the temperature. Let's say we want to vary it by up to ±0.5°C for added realism.
  float randomVariance = (random(100) / 100.0) - 0.5;  // This gives a float between -0.5 to 0.5
  simulatedTemperature += randomVariance;

  return simulatedTemperature;
}