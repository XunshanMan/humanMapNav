  fputs("set terminal x11\n", fp);   /* drawing destination */
  fputs("set grid\n", fp);  /* draw grid */
  fputs("set mouse\n", fp);  /* use mouse */
  fputs("set xlabel \"y [m]\"\n", fp);  /* label of x-axis */
  fputs("set ylabel \"x [m]\"\n", fp);  /* label of y-axis */
  fputs("set xrange [-6:6] reverse\n", fp);  /* range of x-axis */
  fputs("set yrange [-6:6]\n", fp);  /* range of y-axis */
  fputs("set size ratio -1\n", fp);  /* aspect ratio */
  fputs("unset key\n", fp);  /* hide graph legends */

  /* main loop */
  gIsShuttingDown = 0;
  while(!gIsShuttingDown) {
    int i;
    double x, y, rad;

    /* lock buffer */
    ret = S2Sdd_Begin(&urg_buff, &urg_data);

    if(ret > 0) {
      fputs("plot '-'\n", fp);
      for(i = 0; i < urg_data->size; ++i) {
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
      	rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
        x = urg_data->data[i] * cos(rad) / 1000.0;
        y = urg_data->data[i] * sin(rad) / 1000.0;
        fprintf(fp, "%f %f\n", y, x);
      }
      fputs("e\n", fp);

      /* unlock buffer */
      S2Sdd_End(&urg_buff);

      /* wait 90 ms (URG-04LX 1scan = 100 ms) */
      usleep(90000);
    } else if(ret == -1) {
      fprintf(stderr, "ERROR: S2Sdd_Begin\n");
      break;
    } else {
      usleep(100);
    }
  }
